source /opt/ros/humble/setup.bash
_c3_ws=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -f "$_c3_ws/install/local_setup.bash" ]]; then source "$_c3_ws/install/local_setup.bash"; fi
export ROS_DOMAIN_ID=46 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=c3_arm_wsl
unset _c3_ws
