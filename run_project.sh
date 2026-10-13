#!/usr/bin/env bash
# ==============================================================================
# 6-DOF Robotic Sorting Arm - Automated Launcher
# Starts Gazebo Simulation, MoveIt 2 (RViz), and Motion Planning Node
# Supports clean re-running and auto-teardown of previous runs
# ==============================================================================

set -e

echo "=================================================================="
echo "6-DOF Multi-Object Sorting Arm Pipeline"
echo "=================================================================="

# Step 0: Clean up any stale or previously running ROS 2 / Gazebo / MoveIt instances
echo "Stopping any existing simulation and ROS 2 processes..."
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
sleep 2

echo "Starting fresh simulation pipeline..."

# Terminal 1: Gazebo Simulation + ROS Bridge + Vision Node
gnome-terminal --title="1. Gazebo + Vision" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/.ros_overlay/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash
echo '=== Starting Gazebo Simulation and Vision Node ==='
ros2 launch my_sorting_robot sim.launch.py
exec bash
"

echo "Waiting 5 seconds for Gazebo to initialize..."
sleep 5

# Terminal 2: MoveIt 2 + RViz
gnome-terminal --title="2. MoveIt 2 Kinematics & RViz" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/.ros_overlay/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash
echo '=== Starting MoveIt 2 Motion Planning & RViz ==='
RVIZ_CFG=\$(ros2 pkg prefix my_sorting_robot)/share/my_sorting_robot/config/moveit_no_shadow.rviz
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch moveit_resources_panda_moveit_config demo.launch.py rviz_config:=\$RVIZ_CFG
exec bash
"

echo "Waiting 12 seconds for MoveIt controllers & RViz to initialize..."
sleep 12

# Terminal 3: Autonomous Sorting Arm Motion Node
gnome-terminal --title="3. Motion Node" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/.ros_overlay/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash
echo '=== Starting Autonomous Sorting Motion Node ==='
ros2 run my_sorting_robot motion_node
exec bash
"

echo "=================================================================="
echo "All 3 nodes launched successfully in separate terminal windows!"
echo "- To re-run sorting: simply press [ENTER] in Terminal 3 (Motion Node)."
echo "- Or run 'bash ~/Desktop/run_project.sh' anytime to restart cleanly."
echo "=================================================================="
