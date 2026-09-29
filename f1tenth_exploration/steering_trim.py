#!/usr/bin/env python3
"""Calibrate the F1TENTH steering trim so a zero command drives straight.

The servo mapping in vesc.yaml is:

    servo = steering_angle_to_servo_gain * delta + steering_angle_to_servo_offset

The OFFSET is the servo value that corresponds to "wheels straight". It is a
per-car mechanical property — it changes when you adjust the steering linkage,
swap a servo horn, or bend a tie rod — so the stock 0.5304 is only a starting
guess. If it is wrong, a commanded delta of 0 still steers, the car arcs instead
of running straight, and SLAM has to absorb the resulting heading error.

Method: command zero steering at a constant speed, measure the yaw rate the car
actually achieves, and invert the bicycle model to find the steering angle that
produced it.

    delta_err = atan(wheelbase * yaw_rate / speed)
    offset_new = offset_old - gain * delta_err

Runs several passes and averages, alternating nothing else so the only input is
the trim itself.

SAFETY: the car drives FORWARD in short bursts. Needs a clear straight run of a
few metres. Car on the ground, hand on the joystick.

Run with ONLY the hardware up:
    ros2 launch f1tenth_stack bringup_launch.py \
        vesc_config:=~/f1tenth_exploration/vesc_ekf.yaml
    python3 ~/f1tenth_exploration/steering_trim.py
"""
import math
import statistics
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

WHEELBASE = 0.25
GAIN = -1.2135          # steering_angle_to_servo_gain
OFFSET = 0.58           # steering_angle_to_servo_offset currently in vesc_ekf.yaml
                        # (trimmed from the stock 0.5304). Keep this in sync with
                        # the yaml or the suggested correction will be wrong.
SPEED = 0.5             # m/s during a pass — above the motor's start threshold
BURST = 1.6             # s of driving per pass
PASSES = 4


class Trim(Node):
    def __init__(self):
        super().__init__("steering_trim")
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.yaw_rate = 0.0
        self.speed = 0.0
        # /imu/data is the conditioned topic (rad/s, bias removed). Fall back to
        # odom's yaw rate if the conditioner is not running.
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.have_imu = False

    def wait_for_imu(self, timeout=20.0):
        """Block until the conditioned gyro arrives. There is no usable fallback:
        /odom's angular velocity is computed FROM the servo command
        (use_servo_cmd_to_calc_angular_velocity: true in vesc.yaml), so with a
        zero steering command it reports whatever the model says, not what the
        car did. Measuring trim from it is circular."""
        end = time.time() + timeout
        while time.time() < end and not self.have_imu:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.have_imu

    def imu_cb(self, m):
        self.yaw_rate = m.angular_velocity.z
        self.have_imu = True

    def odom_cb(self, m):
        # Only the forward speed is taken from odom; the yaw rate must come
        # from the gyro (see wait_for_imu).
        self.speed = m.twist.twist.linear.x

    def drive(self, speed, secs, collect):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = 0.0
        end = time.time() + secs
        rates, speeds = [], []
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if collect and abs(self.speed) > 0.6 * SPEED:
                rates.append(self.yaw_rate)
                speeds.append(self.speed)
        return rates, speeds


def main():
    rclpy.init()
    n = Trim()
    deltas = []
    try:
        n.drive(0.0, 2.0, False)      # prime odom, let the gyro settle
        if not n.wait_for_imu():
            print("ERROR: no /imu/data. Start the conditioner first:\n"
                  "  python3 ~/f1tenth_exploration/vesc_imu_conditioner.py\n"
                  "Refusing to fall back to /odom: its yaw rate is derived from the\n"
                  "servo COMMAND, so measuring steering trim from it is circular.")
            return
        print("yaw rate source: /imu/data (gyro)")
        print(f"gain={GAIN}  offset={OFFSET}  wheelbase={WHEELBASE} m\n")
        print(f"{'pass':>5} {'speed m/s':>10} {'yaw rate rad/s':>15} {'delta_err rad':>14}")
        for p in range(1, PASSES + 1):
            n.drive(SPEED, 0.8, False)             # spin up, ignore transient
            rates, speeds = n.drive(SPEED, BURST, True)
            n.drive(0.0, 1.5, False)               # stop and settle
            if not rates:
                print(f"{p:5d}   no motion — raise SPEED or check the battery")
                continue
            w = statistics.median(rates)
            v = statistics.median(speeds)
            d = math.atan(WHEELBASE * w / v) if abs(v) > 1e-3 else 0.0
            deltas.append(d)
            print(f"{p:5d} {v:10.3f} {w:15.4f} {d:14.4f}   ({len(rates)} samples)",
                  flush=True)
            print("      turn the car around / reposition, 4 s ...", flush=True)
            n.drive(0.0, 4.0, False)
        if not deltas:
            print("\nNo usable passes.")
            return
        d = statistics.mean(deltas)
        spread = (max(deltas) - min(deltas)) if len(deltas) > 1 else 0.0
        sd = statistics.pstdev(deltas) if len(deltas) > 1 else 0.0
        new_offset = OFFSET - GAIN * d
        print(f"\nmean delta_err  = {d:+.4f} rad ({math.degrees(d):+.2f} deg)")
        print(f"spread          = {spread:.4f} rad ({math.degrees(spread):.2f} deg), "
              f"stdev {sd:.4f}")
        print(f"the car drifts {'LEFT' if d > 0 else 'RIGHT'} on average")
        if spread > 2 * abs(d) and spread > 0.03:
            print("\n*** SPREAD EXCEEDS THE BIAS ***")
            print("The passes are not repeatable, which is the signature of BACKLASH")
            print("in the steering linkage — the wheels settle differently depending")
            print("on which way the servo last moved. A software offset only shifts")
            print("both modes; it cannot remove the slop. Tighten the servo horn,")
            print("tie rods and steering knuckles before trusting any trim value.")
        print(f"\n  steering_angle_to_servo_offset: {OFFSET}  ->  {new_offset:.4f}")
        print("\nEdit that in vesc_ekf.yaml (and f1tenth_stack/config/vesc.yaml),")
        print("then re-run this until delta_err is near zero. Mechanical trim is")
        print("better than software trim: if |delta_err| > ~0.05 rad, straighten")
        print("the steering linkage first rather than papering over it here.")
    finally:
        n.drive(0.0, 1.0, False)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
