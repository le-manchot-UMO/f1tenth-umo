#!/bin/bash
# Find the servo's ACTUAL travel limits, which are not the config's clamps.
#
# vesc.yaml sets servo_min/servo_max to 0.15/0.85, but those are software
# clamps chosen conservatively — the servo itself usually travels further.
# Knowing the real limits matters because the usable steering-trim window is
# derived from them:
#     left  lock: offset - 1.2135*delta_max >= servo_min
#     right lock: offset + 1.2135*delta_max <= servo_max
# Widening servo_max is what buys room to trim out a left veer.
#
# Method: raise the clamps, then step the commanded servo position outward in
# small increments, holding each briefly and returning to centre between steps.
# WATCH THE WHEELS. The real limit is the last value where they still move.
#
# !!! Past the servo's mechanical stop it STALLS: it draws current continuously
# and will overheat and can burn out. Holds are kept to 1.5 s with a return to
# centre between them. STOP THE TEST the moment the wheels stop responding, and
# do not sit on a stalled value.
#
#   servo_limit_test.sh high   # probe upward past 0.85   (buys RIGHT bias)
#   servo_limit_test.sh low    # probe downward past 0.15 (buys LEFT bias)
set -o pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-high}"
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

GAIN=-1.2135
OFF=$(grep -oP 'steering_angle_to_servo_offset:\s*\K[0-9.]+' "$DIR/vesc_ekf.yaml" | tail -1)

if [[ "$MODE" == "high" ]]; then
  TARGETS="0.85 0.88 0.91 0.94"
else
  TARGETS="0.15 0.12 0.09 0.06"
fi

centre() { timeout 1.5 ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
             "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1; }

echo "offset=${OFF} gain=${GAIN}  probing ${MODE}"
echo "WATCH THE WHEELS — note the last value where they still move."
centre
for S in $TARGETS; do
  D=$(python3 -c "print(f'{(${S} - ${OFF})/${GAIN}:.4f}')")
  echo "  servo ${S}  (steering_angle ${D})"
  timeout 1.5 ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
      "{drive: {speed: 0.0, steering_angle: ${D}}}" >/dev/null 2>&1
  centre
done
echo "done — returned to centre"
