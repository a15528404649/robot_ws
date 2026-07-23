#!/usr/bin/env bash
# ROS2 Jazzy replacement for the ROS1 src/sysrun.sh.
# Starts the migrated robot services; detection and WebRTC are intentionally separate.

set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [[ ${1:-} == "--help" ]]; then
  echo "Usage: $0 [navigation_system.launch.py arguments]"
  echo "Example: $0 map:=/absolute/path/to/map.yaml"
  exit 0
fi

source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"

pids=()
cleanup() {
  trap - INT TERM EXIT
  if (( ${#pids[@]} )); then
    kill -INT "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

start_launch() {
  ros2 launch "$@" &
  pids+=($!)
}

# This launch owns base, IMU, lidar, EKF, AMCL and Nav2.
start_launch yzz_robotlaunch navigation_system.launch.py "$@"
# ROS1 robot.launch started GPS and transfer-related functions alongside navigation.
start_launch yzz_gps gps_driver.launch.py
start_launch yzz_battery_adapter battery_adapter.launch.py
start_launch yzz_ptz_camera ptz_control.launch.py
start_launch yzz_transfer yzz_transfer.launch.py start_gps:=false
start_launch yzz_control yzz_control.launch.py

echo "ROS2 system started. Press Ctrl-C to stop all launched services."
echo "Detection and WebRTC are not started by this script."
wait -n "${pids[@]}"
