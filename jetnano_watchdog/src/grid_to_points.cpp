// Copyright 2026 stevej52
// Licensed under the Apache License, Version 2.0. See LICENSE.
//
// grid_to_points: jetnano_bringup/grid_to_points.py in C++ (2026-09-30, Steve: "IMU and
// grid"; the Python version was 4.5-8 % of a core). Same topics, parameters and arithmetic:
//
//   grid_topic (nvblox's occupancy grid, odom frame) -> points_topic: one point per cell at
//     or above occupied_threshold, at z = height, in the grid's frame, for the collision guard
//   map_topic (/map, latched) + SLAM's map->odom from /tf -> map_grid_topic: the camera's
//     grid laid on /map's own cells (occupied / free / unknown -1), at map_grid_hz
//   planner_map_topic: /map with the camera's occupied cells set to 100 - one grid of one
//     size for the planner; /map as it is when the camera grid is older than grid_max_age_s
//
// Why one grid for the planner, and why the warp, is in the Python file's docstring.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class GridToPoints : public rclcpp::Node
{
public:
  GridToPoints()
  : Node("grid_to_points")
  {
    const auto grid_topic = declare_parameter<std::string>("grid_topic", "/nvblox_node/static_occupancy_grid");
    const auto points_topic = declare_parameter<std::string>("points_topic", "/nvblox_node/obstacle_points");
    threshold_ = declare_parameter<int>("occupied_threshold", 65);   // Nav2's static layer threshold (nav2.yaml)
    height_ = declare_parameter<double>("height", 0.20);            // the middle of nvblox's 0.10-0.35 m slice
    const auto map_grid_topic = declare_parameter<std::string>("map_grid_topic", "/nvblox_node/map_grid");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    const double map_grid_hz = declare_parameter<double>("map_grid_hz", 2.0);
    const auto planner_topic = declare_parameter<std::string>("planner_map_topic", "/planner_map");
    max_age_ = declare_parameter<double>("grid_max_age_s", 2.0);
    // nvblox keeps publishing its last grid, with its last stamp, when nothing new is integrated
    // (2026-10-01 19:31: 73 s and counting). The guard refuses a stale source and then holds
    // EVERY command, the lidar's included (drive 16, twelve minutes). A grid older than stale_s
    // goes out as an EMPTY cloud stamped now - the camera's obstacles are gone until it moves
    // again, the lidar still guards - and camera_obstacles/health says "stale N s" so guard_flow
    // caps her speed and the page shows it. Not hidden: degraded, and said. (The Python
    // grid_to_points had this first; the audit of 2026-10-02 found the C++ one runs.)
    stale_s_ = declare_parameter<double>("stale_s", 3.0);
    // Drive 21 (2026-10-02 12:32): a stripe of sunlight on the floor by the east curtain came
    // out of the depth camera as solid obstacle 0.2-0.5 m in front of the mapped wall, the
    // planner's map lost the 0.9 m gap past the box, and "no valid path found" fifteen times.
    // The lidar had the wall where the map has it. A camera cell this close to a mapped wall
    // is the wall (or the sun), not furniture the lidar missed: left out of the planner's
    // map. The local costmap still sees the camera live, so the guard is not affected.
    wall_margin_m_ = declare_parameter<double>("planner_wall_margin_m", 0.40);
    // Drive 22 (13:00, curtain drawn): the sun still came in under the hem, the camera put a
    // blob of "obstacle" in the middle of the hall's end, and the planner failed from the
    // first metre. A camera-only cell is never lethal for the planner any more: it goes into
    // the planner's map at this value (below Nav2's lethal threshold of 65, scaled to a cost
    // by the global static layer's trinary_costmap: false in nav2.yaml), so the planner goes
    // round it when there is a way and through it when there is not; the controller and the
    // collision guard, which see the camera live, keep the last word near it. The lidar's map
    // stays the only thing that forbids. 0 = the camera's cells left out of the planner map.
    camera_value_ = static_cast<int8_t>(declare_parameter<int>("planner_camera_value", 60));
    // Places she must plan round although no sensor sees them reliably, as map boxes
    // [x0, y0, x1, y1, ...], set to 100 on the planner's map. 2026-10-06: the robot vacuum and its
    // dock in the dining-room corner (Steve's turn 4, "where it has always been") - ~10 cm tall,
    // under the lidar's plane and the camera slice's 10 cm floor, and against the wall, so the
    // camera cells it does make fall inside planner_wall_margin_m and are left out; drive 46's
    // parking pushed into it 3-4 times. The camera put it at x 7.45-7.9, y -7.55..-7.9.
    // 2026-10-07 drive 48: she ran straight into it with her nose at x 7.11 (y -7.60), outside the old
    // box (x from 7.25); the camera's blob that lap reached x 7.30, y -7.55..-8.10, and the D435 is
    // blind below ~0.3 m, so the box now starts at x 7.00 and runs to the wall at y -8.15.
    keep_out_ = declare_parameter<std::vector<double>>("keep_out", std::vector<double>{7.00, -8.15, 8.0, -7.30});
    // bumps (jetnano_navigation route_run, via the mission on /bump): a spot she pushed at without
    // moving, a box of +-bump_half_m for bump_keep_s on the planner's map
    bump_half_ = declare_parameter<double>("bump_half_m", 0.10);
    bump_keep_s_ = declare_parameter<double>("bump_keep_s", 600.0);
    health_pub_ =create_publisher<std_msgs::msg::String>("camera_obstacles/health", rclcpp::QoS(1).reliable());

    const auto reliable1 = rclcpp::QoS(1).reliable();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic, reliable1);
    sub_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic, reliable1, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr g) {on_grid(g);});

    if (!map_grid_topic.empty()) {
      const auto latched = rclcpp::QoS(1).reliable().transient_local();
      map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_grid_topic, latched);
      planner_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(planner_topic, latched);
      sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, latched, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
          map_ = m;
          publish_map_grid();                       // a new /map (maybe a new size): the planner's at once
        });
      // /tf comes 140 times a second; only SLAM's map -> odom matters (cheap to skip in C++)
      sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(20).reliable(), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {on_tf(*msg);});
      sub_bump_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "bump", rclcpp::QoS(10).reliable(), [this](geometry_msgs::msg::PointStamped::ConstSharedPtr p) {
          const double x = p->point.x, y = p->point.y;
          bumps_.push_back({x - bump_half_, y - bump_half_, x + bump_half_, y + bump_half_, now().seconds() + bump_keep_s_});
          RCLCPP_INFO(get_logger(), "bump at (%+.2f, %+.2f): a box on the planner's map for %.0f s", x, y, bump_keep_s_);
          publish_map_grid();
        });
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / map_grid_hz), [this]() {publish_map_grid();});
    }
    RCLCPP_INFO(get_logger(), "%s cells >= %d -> %s at z = %.2f%s%s", grid_topic.c_str(), threshold_,
      points_topic.c_str(), height_, map_grid_topic.empty() ? "" : ", and onto the map as ",
      map_grid_topic.c_str());
  }

private:
  static std::string strip(const std::string & f) {return (!f.empty() && f[0] == '/') ? f.substr(1) : f;}

  void on_tf(const tf2_msgs::msg::TFMessage & msg)
  {
    for (const auto & t : msg.transforms) {
      if (strip(t.header.frame_id) == "map" && strip(t.child_frame_id) == "odom") {
        const auto & q = t.transform.rotation;
        tf_x_ = t.transform.translation.x;
        tf_y_ = t.transform.translation.y;
        tf_yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        tf_t_ = now().seconds();
        have_tf_ = true;
      }
    }
  }

  void on_grid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid)
  {
    grid_ = grid;
    // the grid's own measurement stamp decides its age for the planner too (review 2026-10-03:
    // the receipt time said "fresh" while nvblox's stamp stood still); an absurd stamp -> receipt
    const double now_s = now().seconds();
    const double stamp_s = rclcpp::Time(grid->header.stamp).seconds();
    grid_t_ = (stamp_s > 0.0 && std::fabs(now_s - stamp_s) < 60.0) ? stamp_s : now_s;
    const auto & info = grid->info;
    const double res = info.resolution, ox = info.origin.position.x, oy = info.origin.position.y;
    const uint32_t width = info.width;

    // grids from nvblox are axis-aligned with their frame: a cell's centre is a plain offset
    std::vector<float> pts;
    pts.reserve(3 * 512);
    const auto & data = grid->data;
    for (size_t i = 0; i < data.size(); ++i) {
      if (data[i] >= threshold_) {
        pts.push_back(static_cast<float>(ox + (i % width + 0.5) * res));
        pts.push_back(static_cast<float>(oy + (i / width + 0.5) * res));
        pts.push_back(static_cast<float>(height_));
      }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = grid->header;
    const double age = now().seconds() - rclcpp::Time(grid->header.stamp).seconds();
    std_msgs::msg::String health;
    if (age > stale_s_) {
      pts.clear();
      cloud.header.stamp = now();
      health.data = "stale " + std::to_string(static_cast<int>(age)) + " s";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "nvblox grid %.0f s stale: camera obstacles off until it moves again (lidar guards on)", age);
    } else {
      health.data = "ok";
    }
    health_pub_->publish(health);
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size() / 3);
    cloud.fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (int k = 0; k < 3; ++k) {
      cloud.fields[k].name = names[k];
      cloud.fields[k].offset = 4 * k;
      cloud.fields[k].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[k].count = 1;
    }
    cloud.is_bigendian = false;
    cloud.point_step = 12;
    cloud.row_step = 12 * cloud.width;
    cloud.data.resize(pts.size() * sizeof(float));
    std::memcpy(cloud.data.data(), pts.data(), cloud.data.size());
    cloud.is_dense = true;
    pub_->publish(cloud);
  }

  void publish_map_grid()
  {
    if (!map_) {return;}                            // no SLAM: no map to plan on
    const auto & m = map_->info;
    const double t = now().seconds();
    cv::Mat out;                                    // the camera grid on /map's cells, CV_16S, or empty
    if (grid_ && t - grid_t_ <= max_age_ && have_tf_ && t - tf_t_ <= 2.0) {
      const auto & g = grid_->info;
      const double c = std::cos(tf_yaw_), s = std::sin(tf_yaw_);
      const double res = g.resolution, ox = g.origin.position.x, oy = g.origin.position.y;
      const double mr = m.resolution, mx = m.origin.position.x, my = m.origin.position.y;
      // map cell (u, v) centre -> map metres -> odom metres (p_odom = R^T (p_map - t))
      // -> nvblox cell (i, j): one affine map, for warpAffine's inverse mapping
      const double k = mr / res;
      const double ax = mx + 0.5 * mr - tf_x_, ay = my + 0.5 * mr - tf_y_;
      cv::Mat warp = (cv::Mat_<double>(2, 3) <<
        c * k, s * k, (c * ax + s * ay - ox) / res - 0.5,
        -s * k, c * k, (-s * ax + c * ay - oy) / res - 0.5);
      cv::Mat src(g.height, g.width, CV_8S, const_cast<int8_t *>(grid_->data.data()));
      cv::Mat src16;
      src.convertTo(src16, CV_16S);
      cv::warpAffine(src16, out, warp, cv::Size(m.width, m.height),
        cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(-1));
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.frame_id = "map";
      msg.header.stamp = grid_->header.stamp;
      msg.info = m;
      msg.data.resize(static_cast<size_t>(m.width) * m.height);
      for (size_t i = 0; i < msg.data.size(); ++i) {
        msg.data[i] = static_cast<int8_t>(out.at<int16_t>(static_cast<int>(i / m.width), static_cast<int>(i % m.width)));
      }
      map_pub_->publish(msg);
    }
    publish_planner_map(out);
  }

  void publish_planner_map(const cv::Mat & camera)
  {
    // /map, with the cells the camera has occupied (at the threshold) set to 100
    const auto & info = map_->info;
    std::vector<int8_t> data(map_->data.begin(), map_->data.end());
    if (!camera.empty() && camera.rows == static_cast<int>(info.height) && camera.cols == static_cast<int>(info.width)) {
      near_wall_mask();
      camera_value_ = static_cast<int8_t>(get_parameter("planner_camera_value").as_int());   // live: ros2 param set
      size_t left_out = 0;
      for (size_t i = 0; i < data.size(); ++i) {
        const int r = static_cast<int>(i / info.width), c = static_cast<int>(i % info.width);
        if (camera.at<int16_t>(r, c) >= threshold_) {
          if (!near_wall_.empty() && near_wall_.at<uint8_t>(r, c)) {
            ++left_out;
          } else if (data[i] < camera_value_) {
            data[i] = camera_value_;                // expensive, never lethal: see the parameter
          }
        }
      }
      if (left_out > 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
          "%zu camera cell(s) within %.2f m of a mapped wall left out of the planner's map", left_out, wall_margin_m_);
      }
    }
    // the keep-out boxes and the bumps still in force: walls for the planner
    const double t_now = now().seconds();
    bumps_.erase(std::remove_if(bumps_.begin(), bumps_.end(), [t_now](const Box & b) {return b.until <= t_now;}), bumps_.end());
    std::vector<Box> boxes = bumps_;
    for (size_t k = 0; k + 3 < keep_out_.size(); k += 4) {
      boxes.push_back({keep_out_[k], keep_out_[k + 1], keep_out_[k + 2], keep_out_[k + 3], 0.0});
    }
    const double res = info.resolution, mox = info.origin.position.x, moy = info.origin.position.y;
    for (const auto & b : boxes) {
      if (res <= 0.0) {break;}
      const long u0 = std::max(0L, std::lround((std::min(b.x0, b.x1) - mox) / res));
      const long u1 = std::min(static_cast<long>(info.width), std::lround((std::max(b.x0, b.x1) - mox) / res));
      const long v0 = std::max(0L, std::lround((std::min(b.y0, b.y1) - moy) / res));
      const long v1 = std::min(static_cast<long>(info.height), std::lround((std::max(b.y0, b.y1) - moy) / res));
      for (long v = v0; v < v1; ++v) {
        for (long u = u0; u < u1; ++u) {
          data[static_cast<size_t>(v) * info.width + u] = 100;
        }
      }
    }
    const Geometry geometry{info.width, info.height, info.resolution, info.origin.position.x, info.origin.position.y};
    if (have_last_ && geometry == last_geometry_ && data == last_data_) {
      return;                                       // nothing new: no re-inflating the whole house
    }
    have_last_ = true;
    last_geometry_ = geometry;
    last_data_ = data;
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = map_->header.frame_id.empty() ? "map" : map_->header.frame_id;
    msg.header.stamp = now();
    msg.info = info;
    msg.data = std::move(data);
    planner_pub_->publish(msg);
  }

  void near_wall_mask()
  {
    // the cells within planner_wall_margin_m of a /map occupied cell, once per /map message
    if (near_wall_for_ == map_.get()) {return;}
    near_wall_for_ = map_.get();
    const auto & info = map_->info;
    if (wall_margin_m_ <= 0.0 || info.resolution <= 0.0) {
      near_wall_ = cv::Mat();
      return;
    }
    cv::Mat occ(info.height, info.width, CV_8U, cv::Scalar(0));
    for (size_t i = 0; i < map_->data.size(); ++i) {
      if (map_->data[i] >= threshold_) {
        occ.at<uint8_t>(static_cast<int>(i / info.width), static_cast<int>(i % info.width)) = 255;
      }
    }
    const int r = static_cast<int>(std::ceil(wall_margin_m_ / info.resolution));
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1));
    cv::dilate(occ, near_wall_, kernel);
  }

  struct Geometry
  {
    uint32_t w, h; double res, x, y;
    bool operator==(const Geometry & o) const {return w == o.w && h == o.h && res == o.res && x == o.x && y == o.y;}
  };

  struct Box
  {
    double x0, y0, x1, y1, until;
  };
  std::vector<double> keep_out_;
  std::vector<Box> bumps_;
  double bump_half_ = 0.10, bump_keep_s_ = 600.0;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_bump_;
  int threshold_;
  double height_, max_age_, stale_s_, wall_margin_m_;
  int8_t camera_value_ = 60;
  cv::Mat near_wall_;                                // see near_wall_mask()
  const void * near_wall_for_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_, map_;
  double grid_t_ = 0.0;
  bool have_tf_ = false;
  double tf_x_ = 0.0, tf_y_ = 0.0, tf_yaw_ = 0.0, tf_t_ = 0.0;
  bool have_last_ = false;
  Geometry last_geometry_{};
  std::vector<int8_t> last_data_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_, planner_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_, sub_map_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridToPoints>());
  rclcpp::shutdown();
  return 0;
}
