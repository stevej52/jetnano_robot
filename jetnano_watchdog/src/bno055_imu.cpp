// Copyright 2026 stevej52
// Licensed under the Apache License, Version 2.0. See LICENSE.
//
// bno055_imu: the BNO055 driver in C++ - the same node, topics, parameters, register
// sequence and calibration offsets as ros-jazzy-bno055 and jetnano_bringup/bno055_lean.py,
// which this replaces on the robot (2026-09-30, Steve: "IMU and grid"). The Python
// version cost 9-11 % of a core at 100 Hz; most of that was Python itself.
//
//   every 1/data_query_frequency s: one 45-byte read from ACC_DATA_X_LSB, then
//     <prefix>imu          (remapped to imu/data on the robot): fused orientation,
//                          angular velocity, linear acceleration - always
//     <prefix>base         the same reading turned into base_link (imu_mount.py's
//                          arithmetic, base_mount_rpy from the URDF) - while subscribed
//     <prefix>imu_raw, mag, grav, temp - while subscribed, as the stock driver builds them
//   every 1/calib_status_frequency s: <prefix>calib_status, the JSON the stock driver sends
//
// The I2C transfers go through /dev/i2c-N directly (I2C_RDWR), 32 bytes at a time like
// the stock smbus connector. The chip-id check failing at start exits 1, as the stock
// driver does, so the launch's respawn tries again.

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr uint8_t REG_CHIP_ID = 0x00, REG_PAGE_ID = 0x07, REG_ACC_DATA_X_LSB = 0x08,
  REG_CALIB_STAT = 0x35, REG_UNIT_SEL = 0x3B, REG_OPR_MODE = 0x3D, REG_PWR_MODE = 0x3E,
  REG_SYS_TRIGGER = 0x3F, REG_AXIS_MAP_CONFIG = 0x41, REG_ACC_OFFSET_X_LSB = 0x55,
  REG_MAG_OFFSET_X_LSB = 0x5B, REG_GYR_OFFSET_X_LSB = 0x61, REG_ACC_RADIUS_LSB = 0x67,
  REG_MAG_RADIUS_LSB = 0x69;
constexpr uint8_t CHIP_ID = 0xA0, MODE_CONFIG = 0x00, POWER_NORMAL = 0x00;

// imu_mount.py: the fixed turn from the chip's frame into base_link
struct Quat {double w, x, y, z;};
Quat q_from_rpy(double r, double p, double y)
{
  const double cr = std::cos(r / 2), sr = std::sin(r / 2), cp = std::cos(p / 2), sp = std::sin(p / 2),
    cy = std::cos(y / 2), sy = std::sin(y / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}
Quat qmul(const Quat & a, const Quat & b)
{
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
struct Mount
{
  Quat inverse;
  double rot[3][3];
  explicit Mount(const std::vector<double> & rpy)
  {
    const Quat m = q_from_rpy(rpy[0], rpy[1], rpy[2]);     // imu_link in base_link
    inverse = {m.w, -m.x, -m.y, -m.z};
    const double w = m.w, x = m.x, y = m.y, z = m.z;
    const double r[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
      {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
      {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    std::memcpy(rot, r, sizeof(r));
  }
  geometry_msgs::msg::Vector3 vector(const geometry_msgs::msg::Vector3 & v) const
  {
    geometry_msgs::msg::Vector3 o;
    o.x = rot[0][0] * v.x + rot[0][1] * v.y + rot[0][2] * v.z;
    o.y = rot[1][0] * v.x + rot[1][1] * v.y + rot[1][2] * v.z;
    o.z = rot[2][0] * v.x + rot[2][1] * v.y + rot[2][2] * v.z;
    return o;
  }
  void to_base(const sensor_msgs::msg::Imu & src, sensor_msgs::msg::Imu & dst, const std::string & frame) const
  {
    dst.header.stamp = src.header.stamp;
    dst.header.frame_id = frame;
    const Quat q = qmul({src.orientation.w, src.orientation.x, src.orientation.y, src.orientation.z}, inverse);
    dst.orientation.w = q.w; dst.orientation.x = q.x; dst.orientation.y = q.y; dst.orientation.z = q.z;
    dst.orientation_covariance = src.orientation_covariance;
    dst.angular_velocity = vector(src.angular_velocity);
    dst.angular_velocity_covariance = src.angular_velocity_covariance;
    dst.linear_acceleration = vector(src.linear_acceleration);
    dst.linear_acceleration_covariance = src.linear_acceleration_covariance;
  }
};

std::array<double, 9> diagonal(const std::vector<double> & v)
{
  return {v[0], 0.0, 0.0, 0.0, v[1], 0.0, 0.0, 0.0, v[2]};
}
}  // namespace

class I2CDevice
{
public:
  I2CDevice(int bus, int addr)
  : addr_(addr)
  {
    fd_ = ::open(("/dev/i2c-" + std::to_string(bus)).c_str(), O_RDWR);
    if (fd_ < 0) {throw std::runtime_error("cannot open /dev/i2c-" + std::to_string(bus) + ": " + std::strerror(errno));}
  }
  ~I2CDevice() {if (fd_ >= 0) {::close(fd_);}}

  // register read, 32 bytes a transfer like the smbus block read the stock driver uses
  void read(uint8_t reg, uint8_t * out, size_t len)
  {
    size_t done = 0;
    while (done < len) {
      const size_t n = std::min<size_t>(32, len - done);
      uint8_t r = static_cast<uint8_t>(reg + done);
      i2c_msg msgs[2] = {
        {static_cast<uint16_t>(addr_), 0, 1, &r},
        {static_cast<uint16_t>(addr_), I2C_M_RD, static_cast<uint16_t>(n), out + done}};
      i2c_rdwr_ioctl_data xfer = {msgs, 2};
      if (::ioctl(fd_, I2C_RDWR, &xfer) < 0) {
        throw std::runtime_error(std::string("I2C read failed: ") + std::strerror(errno));
      }
      done += n;
    }
  }
  uint8_t read1(uint8_t reg) {uint8_t b = 0; read(reg, &b, 1); return b;}
  void write(uint8_t reg, const uint8_t * data, size_t len)
  {
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    std::memcpy(buf.data() + 1, data, len);
    i2c_msg msg = {static_cast<uint16_t>(addr_), 0, static_cast<uint16_t>(buf.size()), buf.data()};
    i2c_rdwr_ioctl_data xfer = {&msg, 1};
    if (::ioctl(fd_, I2C_RDWR, &xfer) < 0) {
      throw std::runtime_error(std::string("I2C write failed: ") + std::strerror(errno));
    }
  }
  void write1(uint8_t reg, uint8_t v) {write(reg, &v, 1);}

private:
  int fd_ = -1;
  int addr_;
};

class Bno055Node : public rclcpp::Node
{
public:
  Bno055Node()
  : Node("bno055")
  {
    // the stock driver's parameters, same names and defaults (params/NodeParameters.py)
    prefix_ = declare_parameter<std::string>("ros_topic_prefix", "bno055/");
    declare_parameter<std::string>("connection_type", "i2c");
    const int bus = declare_parameter<int>("i2c_bus", 0);
    const int addr = declare_parameter<int>("i2c_addr", 0x28);
    frame_ = declare_parameter<std::string>("frame_id", "bno055");
    const double data_hz = static_cast<double>(declare_parameter<int>("data_query_frequency", 10));
    const double calib_hz = declare_parameter<double>("calib_status_frequency", 0.1);
    operation_mode_ = static_cast<uint8_t>(declare_parameter<int>("operation_mode", 0x0C));
    placement_ = declare_parameter<std::string>("placement_axis_remap", "P1");
    acc_f_ = declare_parameter<double>("acc_factor", 100.0);
    mag_f_ = declare_parameter<double>("mag_factor", 16000000.0);
    gyr_f_ = declare_parameter<double>("gyr_factor", 900.0);
    grav_f_ = declare_parameter<double>("grav_factor", 100.0);
    set_offsets_ = declare_parameter<bool>("set_offsets", false);
    offset_acc_ = declare_parameter<std::vector<int64_t>>("offset_acc", {0xFFEC, 0x00A5, 0xFFE8});
    offset_mag_ = declare_parameter<std::vector<int64_t>>("offset_mag", {0xFFB4, 0xFE9E, 0x027D});
    offset_gyr_ = declare_parameter<std::vector<int64_t>>("offset_gyr", {0x0002, 0xFFFF, 0xFFFF});
    radius_acc_ = declare_parameter<int>("radius_acc", 0x3E8);
    radius_mag_ = declare_parameter<int>("radius_mag", 0x0);
    cov_acc_ = diagonal(declare_parameter<std::vector<double>>("variance_acc", {0.017, 0.017, 0.017}));
    cov_gyr_ = diagonal(declare_parameter<std::vector<double>>("variance_angular_vel", {0.04, 0.04, 0.04}));
    cov_ori_ = diagonal(declare_parameter<std::vector<double>>("variance_orientation", {0.0159, 0.0159, 0.0159}));
    cov_mag_ = diagonal(declare_parameter<std::vector<double>>("variance_mag", {0.0, 0.0, 0.0}));
    // bno055_lean's additions: imu/base in base_link (imu_mount.py)
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    mount_ = std::make_unique<Mount>(declare_parameter<std::vector<double>>(
        "base_mount_rpy", {2.9698, 0.0552, 1.5680}));

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(prefix_ + "imu", 10);
    pub_raw_ = create_publisher<sensor_msgs::msg::Imu>(prefix_ + "imu_raw", 10);
    pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>(prefix_ + "mag", 10);
    pub_grav_ = create_publisher<geometry_msgs::msg::Vector3>(prefix_ + "grav", 10);
    pub_temp_ = create_publisher<sensor_msgs::msg::Temperature>(prefix_ + "temp", 10);
    pub_calib_ = create_publisher<std_msgs::msg::String>(prefix_ + "calib_status", 10);
    pub_base_ = create_publisher<sensor_msgs::msg::Imu>(prefix_ + "base", 10);

    dev_ = std::make_unique<I2CDevice>(bus, addr);
    configure();

    data_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / data_hz), [this]() {read_data();});
    calib_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / calib_hz), [this]() {calib_status();});
  }

private:
  static void pause(int ms) {std::this_thread::sleep_for(std::chrono::milliseconds(ms));}

  void configure()
  {
    RCLCPP_INFO(get_logger(), "Configuring device...");
    try {
      const uint8_t id = dev_->read1(REG_CHIP_ID);
      if (id != CHIP_ID) {throw std::runtime_error("Device ID=" + std::to_string(id) + " is incorrect");}
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Communication error: %s", e.what());
      RCLCPP_ERROR(get_logger(), "Shutting down ROS node...");
      std::exit(1);                                   // as the stock driver; the launch respawns
    }
    const std::map<std::string, std::array<uint8_t, 2>> placements = {
      {"P0", {0x21, 0x04}}, {"P1", {0x24, 0x00}}, {"P2", {0x24, 0x06}}, {"P3", {0x21, 0x02}},
      {"P4", {0x24, 0x03}}, {"P5", {0x21, 0x02}}, {"P6", {0x21, 0x07}}, {"P7", {0x24, 0x05}}};
    try {
      dev_->write1(REG_OPR_MODE, MODE_CONFIG);
      pause(25);                                      // the chip takes 19 ms to change mode
      dev_->write1(REG_PWR_MODE, POWER_NORMAL);
      dev_->write1(REG_PAGE_ID, 0x00);
      dev_->write1(REG_SYS_TRIGGER, 0x00);
      dev_->write1(REG_UNIT_SEL, 0x83);
      const auto p = placements.find(placement_);
      if (p == placements.end()) {
        RCLCPP_WARN(get_logger(), "Unable to set sensor placement configuration.");
      } else {
        dev_->write(REG_AXIS_MAP_CONFIG, p->second.data(), 2);
      }
      RCLCPP_INFO(get_logger(), "Current sensor offsets:");
      print_calib_data();
      if (set_offsets_) {
        set_calib_offsets();
        RCLCPP_INFO(get_logger(), "Successfully configured sensor offsets to:");
        print_calib_data();
      }
      RCLCPP_INFO(get_logger(), "Setting device_mode to %d", operation_mode_);
      dev_->write1(REG_OPR_MODE, operation_mode_);
      pause(25);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "IMU configuration: %s", e.what());
    }
    RCLCPP_INFO(get_logger(), "Bosch BNO055 IMU configuration complete.");
  }

  void write_word(uint8_t reg, int64_t value)
  {
    dev_->write1(reg, static_cast<uint8_t>(value & 0xFF));
    dev_->write1(static_cast<uint8_t>(reg + 1), static_cast<uint8_t>((value >> 8) & 0xFF));
  }

  void set_calib_offsets()
  {
    dev_->write1(REG_OPR_MODE, MODE_CONFIG);
    pause(25);
    for (int i = 0; i < 3; ++i) {write_word(static_cast<uint8_t>(REG_ACC_OFFSET_X_LSB + 2 * i), offset_acc_[i]);}
    write_word(REG_ACC_RADIUS_LSB, radius_acc_);
    for (int i = 0; i < 3; ++i) {write_word(static_cast<uint8_t>(REG_MAG_OFFSET_X_LSB + 2 * i), offset_mag_[i]);}
    write_word(REG_MAG_RADIUS_LSB, radius_mag_);
    for (int i = 0; i < 3; ++i) {write_word(static_cast<uint8_t>(REG_GYR_OFFSET_X_LSB + 2 * i), offset_gyr_[i]);}
  }

  void print_calib_data()
  {
    uint8_t acc[6], mag[6], gyr[6], ar[2], mr[2];
    dev_->read(REG_ACC_OFFSET_X_LSB, acc, 6);
    dev_->read(REG_ACC_RADIUS_LSB, ar, 2);
    dev_->read(REG_MAG_OFFSET_X_LSB, mag, 6);
    dev_->read(REG_MAG_RADIUS_LSB, mr, 2);
    dev_->read(REG_GYR_OFFSET_X_LSB, gyr, 6);
    auto w = [](const uint8_t * b, int i) {return (b[2 * i + 1] << 8) | b[2 * i];};
    RCLCPP_INFO(get_logger(), "\tAccel offsets (x y z): %d %d %d", w(acc, 0), w(acc, 1), w(acc, 2));
    RCLCPP_INFO(get_logger(), "\tAccel radius: %d", w(ar, 0));
    RCLCPP_INFO(get_logger(), "\tMag offsets (x y z): %d %d %d", w(mag, 0), w(mag, 1), w(mag, 2));
    RCLCPP_INFO(get_logger(), "\tMag radius: %d", w(mr, 0));
    RCLCPP_INFO(get_logger(), "\tGyro offsets (x y z): %d %d %d", w(gyr, 0), w(gyr, 1), w(gyr, 2));
  }

  void read_data()
  {
    uint8_t buf[45];
    try {
      dev_->read(REG_ACC_DATA_X_LSB, buf, 45);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiving sensor data failed: %s", e.what());
      return;
    }
    // 22 little-endian int16: 0-2 acc, 3-5 mag, 6-8 gyro, 9-11 euler, 12-15 quaternion w x y z,
    // 16-18 linear acceleration, 19-21 gravity; byte 44 the temperature
    int16_t v[22];
    for (int i = 0; i < 22; ++i) {v[i] = static_cast<int16_t>(buf[2 * i] | (buf[2 * i + 1] << 8));}
    const double qw = v[12], qx = v[13], qy = v[14], qz = v[15];
    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm == 0.0) {return;}                        // an all-zero quaternion: skip the cycle
    const auto stamp = now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame_;
    imu.orientation.x = qx / norm; imu.orientation.y = qy / norm;
    imu.orientation.z = qz / norm; imu.orientation.w = qw / norm;
    imu.orientation_covariance = cov_ori_;
    imu.linear_acceleration.x = v[16] / acc_f_; imu.linear_acceleration.y = v[17] / acc_f_;
    imu.linear_acceleration.z = v[18] / acc_f_;
    imu.linear_acceleration_covariance = cov_acc_;
    imu.angular_velocity.x = v[6] / gyr_f_; imu.angular_velocity.y = v[7] / gyr_f_;
    imu.angular_velocity.z = v[8] / gyr_f_;
    imu.angular_velocity_covariance = cov_gyr_;
    pub_imu_->publish(imu);

    if (pub_base_->get_subscription_count() > 0) {
      sensor_msgs::msg::Imu base;
      mount_->to_base(imu, base, base_frame_);
      pub_base_->publish(base);
    }
    if (pub_raw_->get_subscription_count() > 0) {
      sensor_msgs::msg::Imu raw;
      raw.header = imu.header;
      raw.orientation_covariance = cov_ori_;
      raw.linear_acceleration.x = v[0] / acc_f_; raw.linear_acceleration.y = v[1] / acc_f_;
      raw.linear_acceleration.z = v[2] / acc_f_;
      raw.linear_acceleration_covariance = cov_acc_;
      raw.angular_velocity = imu.angular_velocity;
      raw.angular_velocity_covariance = cov_gyr_;
      pub_raw_->publish(raw);
    }
    if (pub_mag_->get_subscription_count() > 0) {
      sensor_msgs::msg::MagneticField mag;
      mag.header = imu.header;
      mag.magnetic_field.x = v[3] / mag_f_; mag.magnetic_field.y = v[4] / mag_f_; mag.magnetic_field.z = v[5] / mag_f_;
      mag.magnetic_field_covariance = cov_mag_;
      pub_mag_->publish(mag);
    }
    if (pub_grav_->get_subscription_count() > 0) {
      geometry_msgs::msg::Vector3 g;
      g.x = v[19] / grav_f_; g.y = v[20] / grav_f_; g.z = v[21] / grav_f_;
      pub_grav_->publish(g);
    }
    if (pub_temp_->get_subscription_count() > 0) {
      sensor_msgs::msg::Temperature t;
      t.header = imu.header;
      t.temperature = static_cast<double>(buf[44]);
      pub_temp_->publish(t);
    }
  }

  void calib_status()
  {
    uint8_t s;
    try {
      s = dev_->read1(REG_CALIB_STAT);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Receiving calibration status failed: %s", e.what());
      return;
    }
    std_msgs::msg::String msg;
    msg.data = "{\"sys\": " + std::to_string((s >> 6) & 3) + ", \"gyro\": " + std::to_string((s >> 4) & 3) +
      ", \"accel\": " + std::to_string((s >> 2) & 3) + ", \"mag\": " + std::to_string(s & 3) + "}";
    pub_calib_->publish(msg);
  }

  std::string prefix_, frame_, placement_, base_frame_;
  uint8_t operation_mode_;
  double acc_f_, mag_f_, gyr_f_, grav_f_;
  bool set_offsets_;
  std::vector<int64_t> offset_acc_, offset_mag_, offset_gyr_;
  int radius_acc_, radius_mag_;
  std::array<double, 9> cov_acc_, cov_gyr_, cov_ori_, cov_mag_;
  std::unique_ptr<Mount> mount_;
  std::unique_ptr<I2CDevice> dev_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_, pub_raw_, pub_base_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_grav_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_calib_;
  rclcpp::TimerBase::SharedPtr data_timer_, calib_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Bno055Node>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("bno055"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
