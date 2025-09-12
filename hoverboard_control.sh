#!/bin/bash

set -euo pipefail

PASSWORD=${PASSWORD:-Killer64}
CONTAINER=${CONTAINER:-hoverboard}

ros_exec() {
  echo "$PASSWORD" | sudo -S -p '' docker exec "$CONTAINER" bash -lc "source /workspace/install/setup.bash && $*"
}

cmd_vel_once() {
  local lx=${1:-0.0}
  local az=${2:-0.0}
  ros_exec ros2 topic pub --once /cmd_vel geometry_msgs/Twist "'{linear: {x: ${lx}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${az}}}'"
}

cmd_vel_stream() {
  local duration_s=$1
  local rate_hz=${2:-10}
  local lx=${3:-0.0}
  local az=${4:-0.0}
  ros_exec timeout "${duration_s}s" ros2 topic pub -r "$rate_hz" /cmd_vel geometry_msgs/Twist "'{linear: {x: ${lx}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${az}}}'"
}

stop() {
  cmd_vel_once 0.0 0.0
}

forward() {
  local speed=${1:-0.20}
  local duration=${2:-3}
  echo "Forward ${speed} m/s for ${duration}s"
  cmd_vel_stream "$duration" 10 "$speed" 0.0
  stop
}

reverse() {
  local speed=${1:--0.20}
  local duration=${2:-2}
  echo "Reverse ${speed} m/s for ${duration}s"
  cmd_vel_stream "$duration" 10 "$speed" 0.0
  stop
}

rotate() {
  local yawrate=${1:-0.5}
  local duration=${2:-2}
  echo "Rotate in place ${yawrate} rad/s for ${duration}s"
  cmd_vel_stream "$duration" 10 0.0 "$yawrate"
  stop
}

sequence() {
  forward 0.20 3
  rotate 0.5 2
  reverse -0.20 2
  stop
}

usage() {
  cat <<USAGE
Usage: $0 <command> [args]
Commands:
  forward [speed m/s] [duration s]
  reverse [speed m/s] [duration s]
  rotate  [yawrate rad/s] [duration s]
  stop
  sequence   # forward 3s, rotate 2s, reverse 2s, stop

Env vars:
  PASSWORD   sudo password (default: Killer64)
  CONTAINER  docker container name (default: hoverboard)
USAGE
}

case "${1:-}" in
  forward) shift; forward "$@" ;;
  reverse) shift; reverse "$@" ;;
  rotate)  shift; rotate  "$@" ;;
  stop)    shift; stop           ;;
  sequence) shift; sequence      ;;
  *) usage; exit 1 ;;
esac


