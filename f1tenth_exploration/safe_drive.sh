#!/bin/bash
# Drive straight forward, but ONLY if there is genuinely room. Aborts otherwise.
#   safe_drive.sh [speed_mps] [drive_seconds]
#
# Written after a near miss: a clearance check and a drive were chained in one
# command without gating on the result, so the car drove with 0.10 m in front of
# it. The check must be a GATE, not a printout.
#
# Geometry: space_probe reports range from the LASER, which sits 0.27 m ahead of
# base_link, and the car's nose is 0.45 m ahead of base_link. So
#     clearance_ahead_of_bumper = laser_range + 0.27 - 0.45 = laser_range - 0.18
#
# ros2 topic pub needs ~1 s to initialise before it publishes, so the timeout is
# set to drive_seconds + 1.0 to get the requested time actually moving.
set -o pipefail
SPEED="${1:-0.4}"
SECS="${2:-1.2}"
MARGIN="${MARGIN:-0.35}"          # metres to leave spare
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

RANGE=$(timeout 30 python3 "$HOME/f1tenth_exploration/space_probe.py" 2>/dev/null \
        | awk '/FRONT/{print $3}' | grep -oE '[0-9]+\.[0-9]+' | head -1)
[[ -n "$RANGE" ]] || { echo "ABORT: could not read clearance (is /scan up?)"; exit 1; }

NEED=$(python3 -c "print(f'{${SPEED}*${SECS} + ${MARGIN}:.2f}')")
HAVE=$(python3 -c "print(f'{${RANGE} - 0.18:.2f}')")
echo "clearance ahead of bumper: ${HAVE} m   (need ${NEED} m for ${SECS}s @ ${SPEED} m/s + ${MARGIN} margin)"
python3 -c "import sys; sys.exit(0 if ${HAVE} >= ${NEED} else 1)" || {
  echo "ABORT: not enough room. Reposition the car or lower speed/time."; exit 1; }

echo "driving ${SECS}s @ ${SPEED} m/s, steering = 0 ..."
timeout "$(python3 -c "print(${SECS}+1.0)")" ros2 topic pub -r 20 /drive \
    ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {speed: ${SPEED}, steering_angle: 0.0}}" >/dev/null 2>&1
timeout 2 ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1
echo "stopped"
