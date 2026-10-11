#!/usr/bin/env bash
# Запуск Gazebo Harmonic + ArduPilot SITL + ROS 2 (світ maze, дрон iris)
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/ardu_ws/install/ardupilot_gazebo/share:$HOME/ardu_ws/install/ardupilot_gz_description/share"
export SDF_PATH="$SDF_PATH:$HOME/ardu_ws/install/ardupilot_gazebo/share:$HOME/ardu_ws/install/ardupilot_gz_description/share"
"$(dirname "$0")/stop_sim.sh" quiet
exec ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=false
