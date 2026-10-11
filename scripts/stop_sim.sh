#!/usr/bin/env bash
# Зупинка всіх процесів симуляції (перед повторним запуском)
pkill -f "ros2 launch ardupilot_gz_bringup" 2>/dev/null
pkill -f "gz sim|arducopter|mavproxy|micro_ros|robot_state_publisher|rviz2|parameter_bridge" 2>/dev/null
sleep 2
[ "$1" = "quiet" ] || echo "Simulation processes stopped"
exit 0
