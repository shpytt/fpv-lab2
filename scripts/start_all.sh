#!/usr/bin/env bash
# Запускає симуляцію в одній вкладці, чекає готовності автопілота
# і запускає політ у другій
DIR="$(cd "$(dirname "$0")" && pwd)"
gnome-terminal --tab --title="SIM" -- bash -c "$DIR/start_sim.sh; exec bash"
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"

echo "Waiting for the drone (topic /odometry)..."
until ros2 topic list 2>/dev/null | grep -q "^/odometry$"; do sleep 3; done

echo "Waiting for ArduPilot pre-arm check (up to ~10 min)..."
for i in $(seq 1 120); do
  if timeout 15 ros2 service call /ap/v1/prearm_check std_srvs/srv/Trigger 2>/dev/null \
      | grep -q "success=True"; then
    echo "Vehicle is armable"
    break
  fi
  sleep 5
done
sleep 5

gnome-terminal --tab --title="FLIGHT" -- bash -c "$DIR/run_flight.sh target_flight; exec bash"
