# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The BNO055 driver (ros-jazzy-bno055) without the waste.

Every 1/100 s the stock driver reads the chip once, then builds and publishes five
messages: imu/imu (the EKF, the tilt guard and the safety monitor read it, as imu/data)
plus imu/imu_raw, imu/mag, imu/grav and imu/temp, which nothing on Rosie subscribes to.
That cost 19.5 % of a core (2026-09-27), the most of any program outside the camera
container.

This is the stock driver - its node, parameters, calibration offsets and I2C connector -
with the per-cycle read replaced and a lighter executor. The read unpacks the 45 bytes
in one go, builds imu/imu exactly as the stock driver does (same fields, units and
covariances), and builds the other four only while something subscribes to them, so
`ros2 topic echo /imu/mag` still works. It adds imu/base, the same reading in base_link
for the EKF (jetnano_bringup.imu_mount), also only while something reads it. main() is the stock one's timers and error
handling, spun by rclpy's EventsExecutor (see main).
"""

import math
import struct

from bno055 import bno055 as upstream
from bno055 import registers
from bno055.error_handling.exceptions import BusOverRunException
from bno055.sensor.SensorService import SensorService
from geometry_msgs.msg import Vector3
from jetnano_bringup.imu_mount import BASE_MOUNT_RPY, Mount
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.experimental import EventsExecutor
from sensor_msgs.msg import Imu, MagneticField, Temperature

# the 44 bytes from ACC_DATA_X_LSB to GRV_DATA_Z_MSB as little-endian int16:
# 0-2 accelerometer, 3-5 magnetometer, 6-8 gyro, 9-11 euler, 12-15 quaternion w x y z,
# 16-18 linear acceleration, 19-21 gravity; byte 44 is the temperature
_WORDS = struct.Struct('<22h')


def _diagonal(values):
    return np.array([values[0], 0.0, 0.0, 0.0, values[1], 0.0, 0.0, 0.0, values[2]],
                    dtype=np.float64)


class LeanSensorService(SensorService):
    """The stock SensorService; get_sensor_data builds only what someone reads."""

    _fixed = None
    pub_base = None                                 # imu/base (enable_base): off in the tests

    def enable_base(self):
        """Also publish imu/base (jetnano_bringup.imu_mount), while something reads it."""
        node = self.node
        self._base_frame = node.declare_parameter('base_frame', 'base_link').value
        self._mount = Mount(tuple(float(a) for a in
                                  node.declare_parameter('base_mount_rpy', list(BASE_MOUNT_RPY)).value))
        prefix = getattr(self.param, 'ros_topic_prefix', None)
        prefix = prefix.value if prefix is not None else 'imu/'
        self.pub_base = node.create_publisher(Imu, prefix + 'base', 10)

    def _constants(self):
        # the stock NodeParameters holds Parameter snapshots taken at start-up, so
        # these never change while the node runs
        p = self.param
        self._fixed = (
            p.frame_id.value, p.acc_factor.value, p.gyr_factor.value,
            p.mag_factor.value, p.grav_factor.value,
            _diagonal(p.variance_orientation.value), _diagonal(p.variance_acc.value),
            _diagonal(p.variance_angular_vel.value), _diagonal(p.variance_mag.value))
        return self._fixed

    def get_sensor_data(self):
        """Read the chip once; publish imu/imu, and the rest only if subscribed."""
        (frame, acc_f, gyr_f, mag_f, grav_f,
         cov_orientation, cov_acc, cov_gyr, cov_mag) = self._fixed or self._constants()
        buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)
        v = _WORDS.unpack_from(buf)
        stamp = self.node.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = frame
        qw, qx, qy, qz = v[12], v[13], v[14], v[15]
        # an all-zero quaternion raises ZeroDivisionError here, which the stock
        # main() catches and skips the cycle for, as before
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        imu.orientation.x = qx / norm
        imu.orientation.y = qy / norm
        imu.orientation.z = qz / norm
        imu.orientation.w = qw / norm
        imu.orientation_covariance = cov_orientation
        imu.linear_acceleration.x = v[16] / acc_f
        imu.linear_acceleration.y = v[17] / acc_f
        imu.linear_acceleration.z = v[18] / acc_f
        imu.linear_acceleration_covariance = cov_acc
        imu.angular_velocity.x = v[6] / gyr_f
        imu.angular_velocity.y = v[7] / gyr_f
        imu.angular_velocity.z = v[8] / gyr_f
        imu.angular_velocity_covariance = cov_gyr
        self.pub_imu.publish(imu)

        if self.pub_base is not None and self.pub_base.get_subscription_count():
            self.pub_base.publish(self._mount.to_base(imu, Imu(), self._base_frame))

        if self.pub_imu_raw.get_subscription_count():
            raw = Imu()
            raw.header.stamp = stamp
            raw.header.frame_id = frame
            raw.orientation_covariance = cov_orientation
            raw.linear_acceleration.x = v[0] / acc_f
            raw.linear_acceleration.y = v[1] / acc_f
            raw.linear_acceleration.z = v[2] / acc_f
            raw.linear_acceleration_covariance = cov_acc
            raw.angular_velocity = imu.angular_velocity
            raw.angular_velocity_covariance = cov_gyr
            self.pub_imu_raw.publish(raw)
        if self.pub_mag.get_subscription_count():
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = frame
            mag.magnetic_field.x = v[3] / mag_f
            mag.magnetic_field.y = v[4] / mag_f
            mag.magnetic_field.z = v[5] / mag_f
            mag.magnetic_field_covariance = cov_mag
            self.pub_mag.publish(mag)
        if self.pub_grav.get_subscription_count():
            self.pub_grav.publish(Vector3(x=v[19] / grav_f, y=v[20] / grav_f, z=v[21] / grav_f))
        if self.pub_temp.get_subscription_count():
            temp = Temperature()
            temp.header.stamp = stamp
            temp.header.frame_id = frame
            temp.temperature = float(buf[44])
            self.pub_temp.publish(temp)


def main(args=None):
    """Run the stock node and timers, spun by rclpy's EventsExecutor.

    The stock main() spins the default SingleThreadedExecutor, which rebuilds its wait
    set in Python on every wake-up: at 100 Hz that, not the I2C read, was most of the
    driver's CPU (16.7 % of a core with only the unread messages removed).
    """
    # the stock setup() builds `SensorService(...)` by this module-level name
    upstream.SensorService = LeanSensorService
    rclpy.init(args=args)
    node = upstream.Bno055Node()
    try:
        node.setup()
        node.sensor.enable_base()

        def read_data():
            try:
                node.sensor.get_sensor_data()
            except (BusOverRunException, ZeroDivisionError):
                return  # data not ready, or an all-zero quaternion: skip this cycle
            except Exception as e:  # noqa: B902 - as the stock driver: warn, keep going
                node.get_logger().warn(
                    f'Receiving sensor data failed with {type(e).__name__}:"{e}"')

        def log_calibration_status():
            try:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                node.get_logger().warn(
                    f'Receiving calibration status failed with {type(e).__name__}:"{e}"')

        # one thread runs both callbacks, so they never overlap (the stock lock's job)
        node.create_timer(1.0 / float(node.param.data_query_frequency.value), read_data)
        node.create_timer(1.0 / float(node.param.calib_status_frequency.value),
                          log_calibration_status)
        executor = EventsExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
