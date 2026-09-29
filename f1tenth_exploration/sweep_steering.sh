#!/bin/bash
# Sweep the steering left <-> right with the car STATIONARY, to watch the wheels.
#   sweep_steering.sh [cycles] [hold_seconds] [angle_rad]
# Defaults: 3 cycles, 1.5 s per position, 0.26 rad (the mechanical limit).
#
# speed is 0.0 throughout, so the car does not drive. Steering must be commanded
# CONTINUOUSLY: ackermann_mux drops any input silent for 0.2 s, after which the
# servo simply holds its last position and looks broken.
set -o pipefail
CYCLES="${1:-3}"
HOLD="${2:-1.5}"
ANG="${3:-0.26}"
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

OFF=$(grep -oP 'steering_angle_to_servo_offset:\s*\K[0-9.]+' "$HOME/f1tenth_exploration/vesc_ekf.yaml" | tail -1)
GAIN=-1.2135
echo "offset=${OFF}  gain=${GAIN}   servo range clamps to [0.15, 0.85]"
python3 -c "
g=${GAIN}; o=${OFF}; a=${ANG}
for lbl,d in (('LEFT ',a),('CENTRE',0.0),('RIGHT',-a)):
    s=g*d+o
    flag='  <-- CLAMPED' if (s<0.15 or s>0.85) else ''
    print(f'  {lbl}: delta={d:+.3f} rad -> servo={s:.4f}{flag}')
"
pub() {  # pub <angle> <secs>
  timeout "$2" ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
      "{drive: {speed: 0.0, steering_angle: $1}}" >/dev/null 2>&1
}
for ((c=1; c<=CYCLES; c++)); do
  echo "cycle $c/$CYCLES: LEFT";   pub "${ANG}"  "${HOLD}"
  echo "cycle $c/$CYCLES: CENTRE"; pub "0.0"     "${HOLD}"
  echo "cycle $c/$CYCLES: RIGHT";  pub "-${ANG}" "${HOLD}"
  echo "cycle $c/$CYCLES: CENTRE"; pub "0.0"     "${HOLD}"
done
echo "done — steering centred, car never moved"
