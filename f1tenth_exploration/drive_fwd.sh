#!/bin/bash
# Drive straight forward for a short burst, then stop.
#   drive_fwd.sh [speed_mps] [seconds]
# Defaults 0.5 m/s for 2 s (~1 m). Steering command is exactly 0, so whatever
# the car does is the steering trim, not the command.
#
# SAFETY: clear floor ahead. The car stops itself at the end of the burst and
# an explicit zero is published afterwards.
set -o pipefail
SPEED="${1:-0.5}"
SECS="${2:-2.0}"
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

OFF=$(grep -oP 'steering_angle_to_servo_offset:\s*\K[0-9.]+' "$HOME/f1tenth_exploration/vesc_ekf.yaml" | tail -1)
echo "offset in config: ${OFF}   speed ${SPEED} m/s for ${SECS}s (steering command = 0)"

timeout "${SECS}" ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {speed: ${SPEED}, steering_angle: 0.0}}" >/dev/null 2>&1
# leave it stopped: the VESC holds the last speed otherwise
timeout 2 ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1
echo "done — stopped"
