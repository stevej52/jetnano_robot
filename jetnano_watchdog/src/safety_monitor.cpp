// Copyright 2026 stevej52
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// safety_monitor: Rosie's three fast safety watchers in one C++ process.
//
//   ros2 run jetnano_watchdog safety_monitor
//
// They were three Python nodes - jetnano_bringup tilt_guard, motion_check and
// vo_watchdog - listening to the same fast streams (the IMU at 50 Hz, the
// visual odometry at 30 Hz, the drive commands at 20 Hz) and together cost
// about 19 % of a core (measured 2026-09-27), nearly all of it rclpy's
// executor rather than their own arithmetic. Same topics, same thresholds,
// same behaviour, one process; the Python versions stay in jetnano_bringup
// as the fallback (drive.launch.py cpp_safety:=false).
//
// 1. TILT (tilt.enabled): the IMU's orientation, rotated into base_link with the
//    mount looked up once from TF, through a state machine - SAFE, ARMED (past
//    the trigger, waiting out the debounce: a jolt over a rock is not a slope),
//    RECOVERING (reverse on cmd_vel_tilt, which twist_mux ranks above teleop, for
//    at least min and at most max recovery time), LOCKED_OUT (reversing did not
//    help; hand control back until she reads level). Release angles sit below
//    the triggers so it cannot oscillate on the threshold. Roll triggers sooner
//    than pitch: the chassis is longer (0.330 m) than wide (0.230 m). Publishes
//    nothing on cmd_vel_tilt unless recovering, so twist_mux times it out. If
//    the IMU goes quiet it holds the robot still (stop_on_imu_timeout).
//
// 2. MOTION (motion.enabled): told to drive - cmd_vel, after twist_mux and the
//    collision guard, at least min_cmd for window_s - while the odometry moved
//    less than min_move_m or said nothing: the camera is blind (2026-09-26: new
//    D435 firmware, "tracking fine", three metres into the curtains), or the
//    wheels are stuck or in the air. With the lidar odometry running
//    (lidar_odom, lidar_odometry.launch.py) either source seeing her move is
//    enough - she can drive on while the camera's odometry restarts - but the
//    camera seeing next to nothing while the lidar sees her go at least
//    blind_camera_m is the curtains again, and stops her just the same.
//    Raises its own twist_mux lock e_stop_motion,
//    says "nope", and lifts it once every driver has let go (nothing non-zero on
//    cmd_vel_web/teleop/nav/tilt for release_after_s, and at least 2 s).
//    throttle_calibration creeps below the motor's start on purpose and pauses
//    this by publishing motion_check/pause (lapses 2 s after the last one).
//
// 3. VO (vo.enabled, only with the GPU odometry): when /vo goes quiet without
//    anything dying (a camera USB glitch, 2026-09-24) it publishes a zero-velocity
//    odometry on odom_hold after hold_after_s, which the EKF fuses so the pose
//    stays put instead of coasting away - unless the lidar odometry is still
//    talking, which then carries the EKF alone - and after restart_after_s it stops the
//    launch inside the isaac_vo container so the host wrapper respawns it - at
//    most once per restart_min_interval_s, and not in the start-up grace.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <deque>
#include <optional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{

using Quat = std::array<double, 4>;  // x, y, z, w - as ROS orders them

double seconds_since(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

// (roll, pitch) in radians, ZYX; asin clamped because a real IMU's quaternion is
// never exactly unit length
std::pair<double, double> roll_pitch(const Quat & q)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  double sinp = 2.0 * (w * y - z * x);
  sinp = std::max(-1.0, std::min(1.0, sinp));
  return {roll, std::asin(sinp)};
}

Quat quat_from_rpy(double r, double p, double y)
{
  const double cr = std::cos(r / 2), sr = std::sin(r / 2);
  const double cp = std::cos(p / 2), sp = std::sin(p / 2);
  const double cy = std::cos(y / 2), sy = std::sin(y / 2);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

Quat multiply(const Quat & a, const Quat & b)
{
  return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

// The robot's orientation from the IMU's: R_world<-imu * R_base<-imu^-1
Quat orientation_of_base(const Quat & imu, const Quat & mount)
{
  return multiply(imu, {-mount[0], -mount[1], -mount[2], mount[3]});
}

constexpr double kDeg = 180.0 / M_PI;

enum class TiltState { SAFE, ARMED, RECOVERING, LOCKED_OUT };

const char * name_of(TiltState s)
{
  switch (s) {
    case TiltState::SAFE: return "safe";
    case TiltState::ARMED: return "armed";
    case TiltState::RECOVERING: return "recovering";
    default: return "locked_out";
  }
}

}  // namespace

class SafetyMonitor : public rclcpp::Node
{
public:
  SafetyMonitor()
  : Node("safety_monitor")
  {
    // ------------------------------------------------------------------ tilt
    tilt_on_ = declare_parameter("tilt.enabled", true);
    roll_trigger_ = declare_parameter("tilt.roll_trigger_deg", 25.0);
    pitch_trigger_ = declare_parameter("tilt.pitch_trigger_deg", 35.0);
    roll_release_ = declare_parameter("tilt.roll_release_deg", 15.0);
    pitch_release_ = declare_parameter("tilt.pitch_release_deg", 20.0);
    debounce_ = declare_parameter("tilt.debounce_s", 0.2);
    min_recovery_ = declare_parameter("tilt.min_recovery_s", 1.5);
    max_recovery_ = declare_parameter("tilt.max_recovery_s", 5.0);
    reverse_speed_ = std::abs(declare_parameter("tilt.reverse_speed", 0.15));
    const double tilt_rate = declare_parameter("tilt.publish_rate", 20.0);
    imu_timeout_ = declare_parameter("tilt.imu_timeout_s", 1.0);
    stop_on_imu_timeout_ = declare_parameter("tilt.stop_on_imu_timeout", true);
    const bool use_tf = declare_parameter("tilt.use_tf_for_imu_mount", true);
    const auto rpy = declare_parameter("tilt.imu_mount_rpy", std::vector<double>{0.0, 0.0, 0.0});
    base_frame_ = declare_parameter("tilt.base_frame", std::string("base_link"));
    // ---------------------------------------------------------------- motion
    motion_on_ = declare_parameter("motion.enabled", true);
    min_cmd_ = declare_parameter("motion.min_cmd", 0.08);
    window_ = declare_parameter("motion.window_s", 1.5);
    min_move_ = declare_parameter("motion.min_move_m", 0.04);
    release_after_ = declare_parameter("motion.release_after_s", 1.0);
    blind_camera_ = declare_parameter("motion.blind_camera_m", 0.15);
    // ----------------------------------------------------------------- lidar
    lidar_timeout_ = declare_parameter("lidar.timeout_s", 0.5);
    const std::string lidar_topic = declare_parameter("lidar.topic", std::string("lidar_odom"));
    // ------------------------------------------------------------------- vo
    vo_on_ = declare_parameter("vo.enabled", false);
    hold_after_ = declare_parameter("vo.hold_after_s", 0.5);
    restart_after_ = declare_parameter("vo.restart_after_s", 5.0);
    grace_ = declare_parameter("vo.startup_grace_s", 90.0);
    min_interval_ = declare_parameter("vo.restart_min_interval_s", 60.0);
    container_ = declare_parameter("vo.container", std::string("isaac_vo"));
    pattern_ = declare_parameter("vo.launch_pattern", std::string("cuvslam_.*\\.launch\\.py"));
    hold_frame_ = declare_parameter("vo.base_frame", std::string("base_footprint"));

    if (roll_release_ >= roll_trigger_ || pitch_release_ >= pitch_trigger_ ||
      max_recovery_ <= min_recovery_ || debounce_ < 0.0)
    {
      // bad thresholds make the guard useless in a way that is invisible at run time
      RCLCPP_FATAL(get_logger(), "tilt thresholds cannot work: releases must sit below triggers, "
        "max_recovery_s above min_recovery_s, debounce_s not negative");
      throw std::invalid_argument("bad tilt thresholds");
    }

    started_ = std::chrono::steady_clock::now();
    const auto sensor_qos = rclcpp::SensorDataQoS();

    if (tilt_on_) {
      if (use_tf) {
        // the mount is a fixed joint: read it once, then drop the listener (the
        // Python guard kept following /tf at 90 Hz for ever - most of its cost)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      } else {
        mount_ = quat_from_rpy(rpy.at(0), rpy.at(1), rpy.at(2));
        RCLCPP_INFO(get_logger(), "IMU mount from parameter: rpy %.3f %.3f %.3f rad",
          rpy.at(0), rpy.at(1), rpy.at(2));
      }
      tilt_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_tilt", 10);
      tilt_status_pub_ = create_publisher<std_msgs::msg::String>("~/tilt_status", 10);
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(*m);});
      tilt_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / tilt_rate), [this] {tilt_tick();});
      RCLCPP_INFO(get_logger(), "tilt: roll %.0f deg, pitch %.0f deg, reversing at %.2f on cmd_vel_tilt",
        roll_trigger_, pitch_trigger_, reverse_speed_);
    }

    if (motion_on_ || vo_on_) {
      vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "vo", rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_vo(*m);});
      // silent unless lidar_odometry.launch.py runs
      lo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        lidar_topic, rclcpp::QoS(20), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_lidar(*m);});
    }
    if (motion_on_) {
      lock_pub_ = create_publisher<std_msgs::msg::Bool>("e_stop_motion", 10);
      say_pub_ = create_publisher<std_msgs::msg::String>("say", 10);
      state_pub_ = create_publisher<std_msgs::msg::String>("motion_check/state", 10);
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_cmd(*m);});
      pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        "motion_check/pause", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data) {paused_until_ = std::chrono::steady_clock::now() + 2s;}
        });
      for (const char * topic : {"cmd_vel_web", "cmd_vel_teleop", "cmd_vel_nav", "cmd_vel_tilt"}) {
        input_subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
            topic, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
              if (std::abs(m->linear.x) > 0.02 || std::abs(m->angular.z) > 0.02) {
                input_at_ = std::chrono::steady_clock::now();
                any_input_ = true;
              }
            }));
      }
      send_lock(false);
      RCLCPP_INFO(get_logger(), "motion: stops her if told to drive (>= %.2f) for %.1f s while the "
        "odometry moves < %.0f cm; lock e_stop_motion", min_cmd_, window_, min_move_ * 100.0);
    }
    if (vo_on_) {
      hold_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_hold", 10);
      RCLCPP_INFO(get_logger(), "vo: holding the EKF after %.1f s without /vo, restarting the container "
        "launch after %.0f s (grace %.0f s)", hold_after_, restart_after_, grace_);
    }
    if (motion_on_ || vo_on_) {
      timer_ = create_wall_timer(100ms, [this] {tick();});
    }
  }

private:
  // one odometry position, camera's or lidar's, as it arrived
  struct VoSample
  {
    std::chrono::steady_clock::time_point t;
    double x, y;
  };

  // ================================================================== tilt
  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    last_imu_ = now().seconds();
    if (imu_timed_out_) {
      RCLCPP_INFO(get_logger(), "imu/data is publishing again, tilt guard restored");
      imu_timed_out_ = false;
    }
    if (!mount_ && !find_mount(m.header.frame_id)) {
      return;
    }
    const Quat q = orientation_of_base(
      {m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w}, *mount_);
    const auto [roll, pitch] = roll_pitch(q);
    const TiltState before = state_;
    update(roll * kDeg, pitch * kDeg, *last_imu_);
    if (state_ != before) {
      announce(before, state_);
    }
  }

  bool find_mount(const std::string & imu_frame)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(base_frame_, imu_frame, tf2::TimePointZero);
      const auto & r = t.transform.rotation;
      mount_ = Quat{r.x, r.y, r.z, r.w};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "no transform %s <- %s yet, so the tilt guard cannot tell which way is up and is idle. "
        "Is robot_state_publisher running? (%s)", base_frame_.c_str(), imu_frame.c_str(), e.what());
      return false;
    }
    const auto [roll, pitch] = roll_pitch(*mount_);
    RCLCPP_INFO(get_logger(), "IMU mount from TF (%s in %s): roll %.1f deg, pitch %.1f deg; guard active",
      imu_frame.c_str(), base_frame_.c_str(), roll * kDeg, pitch * kDeg);
    tf_listener_.reset();      // it never changes: stop listening
    tf_buffer_.reset();
    return true;
  }

  std::string past_trigger(double roll, double pitch) const
  {
    if (std::abs(roll) >= roll_trigger_) {return "roll";}
    if (std::abs(pitch) >= pitch_trigger_) {return "pitch";}
    return "";
  }

  bool past_release(double roll, double pitch) const
  {
    return std::abs(roll) >= roll_release_ || std::abs(pitch) >= pitch_release_;
  }

  void update(double roll, double pitch, double now_s)
  {
    const std::string axis = past_trigger(roll, pitch);
    switch (state_) {
      case TiltState::SAFE:
        if (!axis.empty()) {
          state_ = TiltState::ARMED;
          armed_at_ = now_s;
          char buf[64];
          std::snprintf(buf, sizeof(buf), "%s %.1f deg", axis.c_str(), axis == "roll" ? roll : pitch);
          last_reason_ = buf;
        }
        break;
      case TiltState::ARMED:
        if (axis.empty()) {
          state_ = TiltState::SAFE;          // a jolt, not a slope
          armed_at_.reset();
        } else if (armed_at_ && now_s - *armed_at_ >= debounce_) {
          state_ = TiltState::RECOVERING;
          recovery_at_ = now_s;
          gave_up_ = false;
        }
        break;
      case TiltState::RECOVERING: {
          const double elapsed = recovery_at_ ? now_s - *recovery_at_ : 0.0;
          if (elapsed >= max_recovery_) {
            // reversing has not helped: hand control back and stay out of the way
            state_ = TiltState::LOCKED_OUT;
            gave_up_ = true;
            recovery_at_.reset();
            armed_at_.reset();
          } else if (elapsed >= min_recovery_ && !past_release(roll, pitch)) {
            state_ = TiltState::SAFE;
            recovery_at_.reset();
            armed_at_.reset();
          }
          break;
        }
      case TiltState::LOCKED_OUT:
        if (!past_release(roll, pitch)) {    // only a genuinely level reading clears it
          state_ = TiltState::SAFE;
          gave_up_ = false;
        }
        break;
    }
  }

  void announce(TiltState before, TiltState after)
  {
    if (after == TiltState::RECOVERING) {
      ++recoveries_;
      RCLCPP_WARN(get_logger(), "tilt limit exceeded (%s), reversing [recovery #%d]",
        last_reason_.c_str(), recoveries_);
    } else if (before == TiltState::RECOVERING && gave_up_) {
      RCLCPP_ERROR(get_logger(), "still tilted after max_recovery_s of reversing; releasing control. "
        "The robot may be stuck, or on a slope it cannot back off.");
    } else if (before == TiltState::RECOVERING) {
      RCLCPP_INFO(get_logger(), "tilt recovered, releasing control");
    }
    std_msgs::msg::String s;
    s.data = std::string(name_of(before)) + " -> " + name_of(after) + ": " + last_reason_;
    tilt_status_pub_->publish(s);
  }

  void tilt_tick()
  {
    if (last_imu_ && now().seconds() - *last_imu_ > imu_timeout_) {
      if (!imu_timed_out_) {
        imu_timed_out_ = true;
        RCLCPP_ERROR(get_logger(), stop_on_imu_timeout_ ?
          "imu/data has stopped; the tilt guard is blind and is HOLDING THE ROBOT STILL. "
          "Set tilt.stop_on_imu_timeout:=false to drive unguarded." :
          "imu/data has stopped; the tilt guard is blind and is NOT protecting the robot.");
      }
      if (stop_on_imu_timeout_) {
        tilt_pub_->publish(geometry_msgs::msg::Twist());    // all zeros: hold still
      }
      return;
    }
    if (state_ == TiltState::RECOVERING) {
      geometry_msgs::msg::Twist t;
      t.linear.x = -reverse_speed_;
      tilt_pub_->publish(t);
    }
    // otherwise publish nothing, and let twist_mux time this input out
  }

  // ============================================================ motion, vo
  void on_cmd(const geometry_msgs::msg::Twist & m)
  {
    const auto now = std::chrono::steady_clock::now();
    cmd_at_ = now;
    if (std::abs(m.linear.x) >= min_cmd_) {
      if (!cmd_since_) {cmd_since_ = now;}
    } else {
      cmd_since_.reset();
    }
  }

  void on_vo(const nav_msgs::msg::Odometry & m)
  {
    const auto now = std::chrono::steady_clock::now();
    vo_.push_back({now, m.pose.pose.position.x, m.pose.pose.position.y});
    while (!vo_.empty() && std::chrono::duration<double>(now - vo_.front().t).count() > 5.0) {
      vo_.pop_front();
    }
    last_vo_ = now;
    on_lidar_ = false;
    if (holding_) {
      holding_ = false;
      RCLCPP_INFO(get_logger(), "/vo is back; releasing the hold");
    }
  }

  void on_lidar(const nav_msgs::msg::Odometry & m)
  {
    const auto now = std::chrono::steady_clock::now();
    lo_.push_back({now, m.pose.pose.position.x, m.pose.pose.position.y});
    while (!lo_.empty() && std::chrono::duration<double>(now - lo_.front().t).count() > 5.0) {
      lo_.pop_front();
    }
    last_lo_ = now;
  }

  bool lidar_alive() const
  {
    return last_lo_ && seconds_since(*last_lo_) < lidar_timeout_;
  }

  struct Track
  {
    std::size_t n;
    double moved;       // farthest from the first sample in the window
  };

  Track track(const std::deque<VoSample> & d, std::chrono::steady_clock::time_point now) const
  {
    Track t{0, 0.0};
    const VoSample * first = nullptr;
    for (const auto & s : d) {
      if (std::chrono::duration<double>(now - s.t).count() > window_) {continue;}
      if (!first) {first = &s;}
      ++t.n;
      t.moved = std::max(t.moved, std::hypot(s.x - first->x, s.y - first->y));
    }
    return t;
  }

  void tick()
  {
    if (motion_on_) {motion_tick();}
    if (vo_on_) {vo_tick();}
  }

  void motion_tick()
  {
    const auto now = std::chrono::steady_clock::now();
    if (locked_) {
      const bool drivers_let_go = !any_input_ || seconds_since(input_at_) >= release_after_;
      if (drivers_let_go && seconds_since(locked_at_) >= 2.0) {
        locked_ = false;
        send_lock(false);
        RCLCPP_INFO(get_logger(), "released: nobody is driving now");
        publish_state("ok");
      } else if (seconds_since(lock_sent_) >= 1.0) {
        send_lock(true);
      }
      return;
    }
    if (!get_parameter("motion.enabled").as_bool() || now < paused_until_) {
      return;
    }
    if (!cmd_since_ || seconds_since(cmd_at_) > 0.3 || seconds_since(*cmd_since_) < window_) {
      return;
    }
    const Track vo = track(vo_, now);
    const Track lo = track(lo_, now);
    const bool has_vo = vo.n >= 5, has_lo = lo.n >= 5;
    char reason[160];
    if (!has_vo && !has_lo) {
      std::snprintf(reason, sizeof(reason), "no odometry for %.1f s", window_);
    } else if (has_vo && has_lo && lo.moved >= blind_camera_ && vo.moved < 0.3 * lo.moved) {
      // the curtains (2026-09-26): the camera "tracking fine" and seeing nothing move
      std::snprintf(reason, sizeof(reason), "the camera's odometry moved %.1f cm while the lidar's moved "
        "%.0f cm in %.1f s - the camera is blind", vo.moved * 100.0, lo.moved * 100.0, window_);
    } else {
      const double moved = std::max(has_vo ? vo.moved : 0.0, has_lo ? lo.moved : 0.0);
      if (moved >= min_move_) {
        if (has_vo && has_lo && vo.moved >= blind_camera_ && lo.moved < 0.3 * vo.moved) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "the camera's odometry moved %.0f cm but "
            "the lidar's %.1f cm: the lidar odometry has lost her (a long bare wall, glass, open space?)",
            vo.moved * 100.0, lo.moved * 100.0);
        }
        return;
      }
      std::snprintf(reason, sizeof(reason), "the odometry moved %.1f cm in %.1f s", moved * 100.0, window_);
    }
    locked_ = true;
    locked_at_ = now;
    ++events_;
    cmd_since_.reset();
    send_lock(true);
    std_msgs::msg::String say;
    say.data = "nope";
    say_pub_->publish(say);
    publish_state(std::string("stopped: told to drive, ") + reason);
    RCLCPP_WARN(get_logger(), "STOPPED her: told to drive for %.1f s but %s - odometry blind, wheels "
      "stuck or in the air (event %d); released when nobody is driving", window_, reason, events_);
  }

  void vo_tick()
  {
    const auto now = std::chrono::steady_clock::now();
    const double since_start = seconds_since(started_);
    const double silent = last_vo_ ? seconds_since(*last_vo_) : since_start;
    const bool vo_gone = silent > hold_after_ && (last_vo_ || since_start > grace_);
    if (vo_gone && lidar_alive()) {
      // the EKF still has a real measurement of her motion: a zero here would fight it
      if (holding_) {
        holding_ = false;
        RCLCPP_INFO(get_logger(), "the lidar odometry is talking again; releasing the hold");
      }
      if (!on_lidar_) {
        on_lidar_ = true;
        RCLCPP_WARN(get_logger(), "/vo silent for %.1f s: the lidar odometry carries the EKF", silent);
      }
    } else if (vo_gone) {
      if (!holding_) {
        holding_ = true;
        RCLCPP_WARN(get_logger(), "/vo silent for %.1f s: holding the EKF still", silent);
      }
      nav_msgs::msg::Odometry m;
      m.header.stamp = this->now();
      m.header.frame_id = "odom";
      m.child_frame_id = hold_frame_;
      m.twist.covariance[0] = 1e-3;     // vx
      m.twist.covariance[7] = 1e-3;     // vy
      hold_pub_->publish(m);
    }
    if (silent > restart_after_ && since_start > grace_ &&
      (!last_restart_ || seconds_since(*last_restart_) > min_interval_))
    {
      last_restart_ = now;
      RCLCPP_ERROR(get_logger(), "/vo silent for %.0f s: stopping the launch in %s so it is respawned",
        silent, container_.c_str());
      // off the executor: docker can take a while, and a hung docker must not stop the checks
      const std::string cmd = "timeout 10 docker exec " + container_ + " pkill -INT -f '" + pattern_ +
        "' >/dev/null 2>&1";
      std::thread([cmd, logger = get_logger()] {
          if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(logger, "could not reach the container");
          }
        }).detach();
    }
  }

  void send_lock(bool value)
  {
    std_msgs::msg::Bool b;
    b.data = value;
    lock_pub_->publish(b);
    lock_sent_ = std::chrono::steady_clock::now();
  }

  void publish_state(const std::string & text)
  {
    std_msgs::msg::String s;
    s.data = text;
    state_pub_->publish(s);
  }

  // tilt
  bool tilt_on_{true}, stop_on_imu_timeout_{true}, imu_timed_out_{false}, gave_up_{false};
  double roll_trigger_, pitch_trigger_, roll_release_, pitch_release_;
  double debounce_, min_recovery_, max_recovery_, reverse_speed_, imu_timeout_;
  std::string base_frame_, last_reason_;
  std::optional<Quat> mount_;
  std::optional<double> last_imu_, armed_at_, recovery_at_;
  TiltState state_{TiltState::SAFE};
  int recoveries_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tilt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tilt_status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr tilt_timer_;
  // motion
  bool motion_on_{true}, locked_{false}, any_input_{false};
  double min_cmd_, window_, min_move_, release_after_, blind_camera_;
  int events_{0};
  std::optional<std::chrono::steady_clock::time_point> cmd_since_;
  std::chrono::steady_clock::time_point cmd_at_{}, input_at_{}, locked_at_{}, lock_sent_{}, paused_until_{};
  std::deque<VoSample> vo_, lo_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr say_pub_, state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> input_subs_;
  // vo
  bool vo_on_{false}, holding_{false}, on_lidar_{false};
  double hold_after_, restart_after_, grace_, min_interval_;
  std::string container_, pattern_, hold_frame_;
  std::chrono::steady_clock::time_point started_;
  std::optional<std::chrono::steady_clock::time_point> last_vo_, last_restart_, last_lo_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_, lo_sub_;
  // lidar
  double lidar_timeout_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr hold_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
