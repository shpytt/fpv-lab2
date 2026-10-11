#!/usr/bin/env bash
# Перезбірка пакета fpv_lab2 (потрібна після кожної зміни коду)
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"
cd "$HOME/fpv_labs" && colcon build --symlink-install --packages-select fpv_lab2
