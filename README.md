http://13.125.65.10:5022/


# if you want ROS2 Foxy
alias cm2='cd ~/Desktop/ros2_ws && colcon build'
alias cs2='cd ~/Desktop/ros2_ws/src'

alias cm_dubal='cd ~/Desktop/dubal_ws && colcon build'
alias cs_dubal='cd ~/Desktop/dubal_ws/src'
alias du='ros2 launch controller dubal_no_eye.py'
alias du_eye='ros2 launch controller dubal_jjas_eye.py'
alias rr='rm -rf build/ install/ log/ && cd /home/ryung/Desktop/buf/build/controller && rm CMakeCache.txt'

echo "ROS2 Foxy running.."
source /opt/ros/foxy/setup.bash

export ROS_DOMAIN_ID=13
#source ~/Desktop/ros2_ws/install/setup.bash
source ~/Desktop/dubal_ws/install/setup.bash
