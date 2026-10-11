#!/usr/bin/env bash
# Запуск польоту. Використання: ./run_flight.sh [target_flight|flight_test]
NODE="${1:-target_flight}"
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"
source "$HOME/fpv_labs/install/setup.bash"
mkdir -p "$HOME/lab2_report"
ros2 run fpv_lab2 "$NODE" 2>&1 | tee "$HOME/lab2_report/${NODE}_log.txt"
