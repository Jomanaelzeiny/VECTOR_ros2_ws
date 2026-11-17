#!/usr/bin/env bash
# Drive the real car STRAIGHT, then stop.
# Usage: ./straight.sh [speed_m_s] [seconds]     e.g. ./straight.sh 0.2 3
SPEED=${1:-0.2}
SECS=${2:-3}
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
TOPIC=/car_01/mr/cmd_vel
STOP='{linear: {x: 0.0}, angular: {z: 0.0}}'

# Ctrl+C at any time -> stop the car
trap 'ros2 topic pub -t 10 -r 20 $TOPIC geometry_msgs/msg/Twist "$STOP" >/dev/null; echo "STOPPED (Ctrl+C)"; exit 0' INT

echo "Driving STRAIGHT at $SPEED m/s for $SECS s ... (Ctrl+C = stop)"
timeout "$SECS" ros2 topic pub -r 10 $TOPIC geometry_msgs/msg/Twist \
  "{linear: {x: $SPEED}, angular: {z: 0.0}}" >/dev/null

# send stop several times so it is surely received
ros2 topic pub -t 10 -r 20 $TOPIC geometry_msgs/msg/Twist "$STOP" >/dev/null
echo "STOPPED"
