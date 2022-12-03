#!/bin/bash -e

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch ai-course-project main.launch.py $@

