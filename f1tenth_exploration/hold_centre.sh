#!/bin/bash
# Hold the steering at commanded centre so the linkage can be adjusted against a
# live, powered servo.
#   hold_centre.sh [seconds]      (default 180)
#
# Why hold rather than send once: ackermann_mux drops an input that goes silent
# for 0.2 s, after which the servo merely holds its last position with nothing
# commanding it. To trim a horn or tie rod you want the servo actively driven to
# centre, so it resists and you are setting the true neutral.
#
# Procedure:
#   1. Run this. The servo goes to centre (servo value = the offset) and stays.
#   2. With it held, adjust so the WHEELS point dead ahead:
#        - coarse: pull the servo horn and re-seat it one spline over
#        - fine:   change tie-rod length
#   3. Stop this, reset the offset to stock 0.5304, and re-run a straight drive.
#
# Speed is 0.0 throughout; the car does not drive.
set -o pipefail
SECS="${1:-180}"
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
OFF=$(grep -oP 'steering_angle_to_servo_offset:\s*\K[0-9.]+' "$HOME/f1tenth_exploration/vesc_ekf.yaml" | tail -1)
echo "holding steering at centre for ${SECS}s (servo = ${OFF}); car will not move"
echo "adjust the servo horn / tie rod NOW so the wheels point straight ahead"
timeout "${SECS}" ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1
echo "hold released"
