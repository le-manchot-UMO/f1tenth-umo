#!/usr/bin/env python3
"""Make the VESC IMU usable by robot_localization.

`/sensors/imu/raw` comes straight off the VESC with no conditioning at all
(vesc_driver.cpp copies the packet fields into a sensor_msgs/Imu verbatim), so
it breaks robot_localization in three separate ways:

  1. `header.frame_id` is the empty string, so the EKF cannot look up a TF for
     the sensor and drops every message.
  2. Every covariance matrix is all-zero. robot_localization reads a zero
     covariance as "infinitely certain" and the filter degenerates.
  3. The units are VESC firmware units, not REP-145 SI: the gyro reports deg/s
     and the accelerometer reports g. Measured on this car at rest,
     linear_acceleration.z == 1.01, which is 1 g rather than 9.81 m/s^2.

This node republishes a corrected sensor_msgs/Imu on `/imu/data`, and also
removes the gyro's zero-rate bias, which is the dominant source of yaw drift.
The bias is measured from the first `calibration_samples` messages, so THE CAR
MUST BE STATIONARY FOR THE FIRST FEW SECONDS AFTER LAUNCH. Until calibration
finishes nothing is published, which conveniently makes the EKF (and therefore
Nav2) wait rather than start on a biased gyro.

Frames: the VESC sits flat on the chassis and reads +1 g on z at rest, so its
axes already agree with base_link and the transform to `imu_link` is identity.
Remount the VESC rotated and you must change that static transform (published
by exploration_launch.py), not this file.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

DEG_TO_RAD = math.pi / 180.0
G_TO_MS2 = 9.80665


class VescImuConditioner(Node):
    def __init__(self):
        super().__init__("vesc_imu_conditioner")

        self.declare_parameter("input_topic", "/sensors/imu/raw")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        # Set false if a future VESC firmware already emits rad/s and m/s^2.
        self.declare_parameter("convert_units", True)
        self.declare_parameter("calibration_samples", 200)  # ~4 s at 50 Hz
        # Diagonal covariances. The VESC IMU is a cheap MEMS part; these are
        # deliberately loose so the EKF leans on it for yaw rate without
        # trusting it absolutely.
        self.declare_parameter("angular_velocity_variance", 0.002)
        self.declare_parameter("linear_acceleration_variance", 0.05)
        # Negative marks orientation as unavailable per sensor_msgs/Imu, which
        # tells robot_localization to ignore it outright.
        self.declare_parameter("orientation_unavailable", True)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.convert = self.get_parameter("convert_units").value
        self.n_calib = int(self.get_parameter("calibration_samples").value)
        self.gyro_var = float(self.get_parameter("angular_velocity_variance").value)
        self.accel_var = float(self.get_parameter("linear_acceleration_variance").value)
        self.no_orientation = self.get_parameter("orientation_unavailable").value

        self.samples = []
        self.bias = None

        # The VESC driver publishes with the default reliable QoS.
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Imu, self.out_topic, qos)
        self.sub = self.create_subscription(Imu, self.in_topic, self.cb, qos)

        self.get_logger().info(
            f"Conditioning {self.in_topic} -> {self.out_topic} "
            f"(frame_id={self.frame_id}, convert_units={self.convert}). "
            f"KEEP THE CAR STILL: calibrating gyro bias over {self.n_calib} samples."
        )

    def cb(self, msg: Imu):
        gx, gy, gz = (msg.angular_velocity.x,
                      msg.angular_velocity.y,
                      msg.angular_velocity.z)
        ax, ay, az = (msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z)

        if self.convert:
            gx, gy, gz = gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD
            ax, ay, az = ax * G_TO_MS2, ay * G_TO_MS2, az * G_TO_MS2

        if self.bias is None:
            self.samples.append((gx, gy, gz))
            if len(self.samples) >= self.n_calib:
                n = float(len(self.samples))
                self.bias = tuple(sum(s[i] for s in self.samples) / n for i in range(3))
                self.get_logger().info(
                    "Gyro bias (rad/s): x=%.5f y=%.5f z=%.5f — publishing now."
                    % self.bias
                )
            return

        out = Imu()
        # Reuse the driver's stamp: it is set at packet-receive time, and the
        # EKF needs the sensor's own timeline, not ours.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        out.angular_velocity.x = gx - self.bias[0]
        out.angular_velocity.y = gy - self.bias[1]
        out.angular_velocity.z = gz - self.bias[2]

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az

        if self.no_orientation:
            out.orientation_covariance = [-1.0] + [0.0] * 8
        else:
            out.orientation = msg.orientation
            out.orientation_covariance = [0.05, 0.0, 0.0,
                                          0.0, 0.05, 0.0,
                                          0.0, 0.0, 0.05]

        out.angular_velocity_covariance = [self.gyro_var, 0.0, 0.0,
                                           0.0, self.gyro_var, 0.0,
                                           0.0, 0.0, self.gyro_var]
        out.linear_acceleration_covariance = [self.accel_var, 0.0, 0.0,
                                              0.0, self.accel_var, 0.0,
                                              0.0, 0.0, self.accel_var]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VescImuConditioner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
