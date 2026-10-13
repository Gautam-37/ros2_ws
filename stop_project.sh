#!/usr/bin/env bash
# ==============================================================================
# Stops all Robotic Arm simulation and ROS 2 nodes cleanly
# ==============================================================================

echo "Stopping 6-DOF Robotic Sorting Arm pipeline..."
pkill -9 -f "ros2 launch my_sorting_robot" 2>/dev/null || true
pkill -9 -f "ros2 launch moveit_resources" 2>/dev/null || true
pkill -9 -f "ros2 run my_sorting_robot" 2>/dev/null || true
pkill -9 -f "my_sorting_robot" 2>/dev/null || true
pkill -9 -f "moveit_resources" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "move_group" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
echo "All simulation and ROS 2 processes stopped."
