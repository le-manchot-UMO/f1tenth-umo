#!/bin/bash
# Set the steering trim and restart the VESC driver so it takes effect.
#   set_offset.sh <offset>
# ackermann_to_vesc reads steering_angle_to_servo_offset once at startup into a
# member variable, so `ros2 param set` alone does NOT change behaviour — the
# node has to be restarted. That is what this does.
#
# Higher offset steers LEFT on this car (determined empirically; it is the
# opposite of what the negative gain suggests).
set -o pipefail
[[ $# -eq 1 ]] || { echo "usage: $0 <offset>  (e.g. 0.5420)"; exit 2; }
NEW="$1"
YAML="$HOME/f1tenth_exploration/vesc_ekf.yaml"
OLD=$(grep -oP 'steering_angle_to_servo_offset:\s*\K[0-9.]+' "$YAML" | tail -1)
sed -i "s/steering_angle_to_servo_offset: ${OLD}/steering_angle_to_servo_offset: ${NEW}/" "$YAML"
echo "offset ${OLD} -> ${NEW}, restarting driver..."

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
tmux kill-session -t hw 2>/dev/null
sleep 2
tmux new-session -d -s hw \
  "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; \
   source /opt/ros/humble/setup.bash; source $HOME/f1tenth_ws/install/setup.bash; \
   ros2 launch f1tenth_stack bringup_launch.py vesc_config:=${YAML} > /tmp/hw.log 2>&1"
sleep 18
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
echo -n "driver now reports: "
timeout 10 ros2 param get /ackermann_to_vesc_node steering_angle_to_servo_offset 2>&1 | tail -1
