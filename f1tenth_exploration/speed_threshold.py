#!/usr/bin/env python3
"""Find the lowest commanded speed that actually turns this car's wheels.

Why you need this: the VESC will not start a sensorless motor at very low eRPM.
On the reference car Nav2 commanded 0.19 m/s (~880 eRPM) and the VESC reported
current_motor = -0.03 A with speed = 0.0 — the command was accepted and the car
did not move. Nav2 then aborts every goal on a progress-checker timeout, which
looks like a planning bug and is really a motor threshold.

Feed the result into:
  * cmd_vel_to_ackermann.py   -> min_move_speed
  * nav2_params.yaml          -> regulated_linear_scaling_min_speed,
                                 min_approach_linear_velocity

SAFETY: the car drives FORWARD in short bursts, stopping between each, up to
0.90 m/s. At the default 0.8 s burst that is at most ~0.72 m of travel per step,
but confirm your actual forward clearance with space_probe.py first. Car on the
ground, hand on the joystick.

Run with ONLY the hardware up (no Nav2, or the bridge will fight you):
    ros2 launch f1tenth_stack bringup_launch.py \
        vesc_config:=~/f1tenth_exploration/vesc_ekf.yaml \
        sensors_config:=~/f1tenth_exploration/sensors_masked.yaml
    python3 ~/f1tenth_exploration/speed_threshold.py
"""
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

STEPS = [0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.75, 0.90]
ERPM_GAIN = 4614.0          # speed_to_erpm_gain from vesc.yaml
MOVED = 0.05                # m/s measured that counts as real motion
# Burst length. Distance covered is roughly BURST * speed, so keep
# BURST * max(STEPS) comfortably under your forward clearance:
# 0.8 s * 0.90 m/s = 0.72 m. Check clearance first with space_probe.py.
BURST = 0.8                 # s


class SpeedThreshold(Node):
    def __init__(self):
        super().__init__("speed_threshold")
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.measured = 0.0
        self.create_subscription(Odometry, "/odom", self.cb, 10)

    def cb(self, msg):
        self.measured = msg.twist.twist.linear.x

    def hold(self, speed, secs):
        """Publish a constant command for secs, return peak |measured speed|."""
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = 0.0
        end = time.time() + secs
        peak = 0.0
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            peak = max(peak, abs(self.measured))
        return peak


def main():
    rclpy.init()
    n = SpeedThreshold()
    try:
        print("priming odometry (stationary, 2 s) ...")
        n.hold(0.0, 2.0)
        print(f"\n{'cmd m/s':>9} {'eRPM':>8} {'measured m/s':>13}   moved?")
        first = None
        for v in STEPS:
            peak = n.hold(v, BURST)
            n.hold(0.0, 1.2)                      # stop between steps
            moved = peak > MOVED
            print(f"{v:9.2f} {v*ERPM_GAIN:8.0f} {peak:13.3f}   "
                  f"{'YES' if moved else 'no'}", flush=True)
            if moved:
                first = v
                break
        print("")
        if first is None:
            print("No motion up to 0.90 m/s. Check the traction battery, the motor "
                  "connector, and /sensors/core fault_code.")
        else:
            print(f"Lowest speed that moved the car: {first:.2f} m/s "
                  f"({first*ERPM_GAIN:.0f} eRPM)")
            print(f"Set min_move_speed to about {first + 0.05:.2f} m/s "
                  f"(one step of margin).")
    finally:
        n.hold(0.0, 1.0)                          # leave it stopped
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
