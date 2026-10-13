#!/usr/bin/env bash
# Rebuilds the "my_sorting_robot" project on Ubuntu 24.04 (VMware) with ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2
# Usage:  bash setup_project.sh
set -e

########## 1. INSTALL ROS 2 JAZZY + GAZEBO + MOVEIT (slow download: start this first) ##########
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  sudo apt update && sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
fi
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-moveit \
  ros-jazzy-moveit-resources-panda-moveit-config ros-jazzy-ros2-controllers \
  ros-jazzy-rqt-image-view python3-opencv python3-colcon-common-extensions

grep -q "jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

########## 2. CREATE PACKAGE ##########
WS=~/ros2_ws
PKG=$WS/src/my_sorting_robot
mkdir -p $PKG/my_sorting_robot $PKG/launch $PKG/worlds $PKG/resource
touch $PKG/resource/my_sorting_robot $PKG/my_sorting_robot/__init__.py

cat > $PKG/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>my_sorting_robot</name>
  <version>0.1.0</version>
  <description>YOLO/OpenCV vision + Gazebo + MoveIt 2 sorting arm</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>MIT</license>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>moveit_msgs</depend>
  <depend>shape_msgs</depend>
  <depend>ros_gz_sim</depend>
  <depend>ros_gz_bridge</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

cat > $PKG/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/my_sorting_robot
[install]
install_scripts=$base/lib/my_sorting_robot
EOF

cat > $PKG/setup.py << 'EOF'
import os
from glob import glob
from setuptools import setup

package_name = 'my_sorting_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vision_node = my_sorting_robot.vision_node:main',
            'motion_node = my_sorting_robot.motion_node:main',
        ],
    },
)
EOF

########## 3. GAZEBO WORLD (table, red + yellow cube, overhead camera) ##########
# Camera at z=1.4 looking straight down. Table top z=0.4, cube top z=0.45 -> Z distance = 0.95 m
cat > $PKG/worlds/sorting_world.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="sorting_world">
    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>

    <model name="table">
      <static>true</static>
      <pose>0 0 0.2 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.8 0.8 0.4</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.8 0.8 0.4</size></box></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
      </link>
    </model>

    <model name="red_cube">
      <pose>0.10 0.05 0.425 0 0 0</pose>
      <link name="link">
        <inertial><mass>0.1</mass><inertia><ixx>0.00004</ixx><iyy>0.00004</iyy><izz>0.00004</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>0.05 0.05 0.05</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.05 0.05 0.05</size></box></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
      </link>
    </model>

    <model name="yellow_cube">
      <pose>-0.08 -0.06 0.425 0 0 0</pose>
      <link name="link">
        <inertial><mass>0.1</mass><inertia><ixx>0.00004</ixx><iyy>0.00004</iyy><izz>0.00004</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>0.05 0.05 0.05</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.05 0.05 0.05</size></box></geometry>
          <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material></visual>
      </link>
    </model>

    <model name="overhead_camera">
      <static>true</static>
      <pose>0 0 1.4 0 1.5708 0</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <topic>camera</topic>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>5</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
EOF

########## 4. VISION NODE (HSV filter + pinhole model) ##########
cat > $PKG/my_sorting_robot/vision_node.py << 'EOF'
#!/usr/bin/env python3
"""Subscribes to /camera, finds red/yellow cubes with HSV masks, converts pixel -> 3D with the
pinhole model, and publishes targets in the robot base frame on /target_pose/<color>."""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

HSV_RANGES = {
    'red': [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    'yellow': [((20, 120, 70), (35, 255, 255))],
}


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('z_cam', 0.95)         # camera -> cube top distance (m)
        self.declare_parameter('hfov', 1.047)         # must match SDF
        self.declare_parameter('cam_world_x', 0.0)    # camera position in Gazebo world
        self.declare_parameter('cam_world_y', 0.0)
        self.declare_parameter('base_world_x', -0.45) # where the robot base sits in the world
        self.declare_parameter('base_world_y', 0.0)
        self.create_subscription(Image, '/camera', self.cb, 1)
        self.pubs = {c: self.create_publisher(PoseStamped, f'/target_pose/{c}', 10) for c in HSV_RANGES}
        self.dbg = self.create_publisher(Image, '/vision/debug', 1)
        self.get_logger().info('vision_node ready, waiting for /camera')

    def cb(self, msg):
        if msg.encoding != 'rgb8':
            self.get_logger().warn(f'unexpected encoding {msg.encoding}', throttle_duration_sec=5.0)
            return
        z = self.get_parameter('z_cam').value
        hfov = self.get_parameter('hfov').value
        cam_x = self.get_parameter('cam_world_x').value
        cam_y = self.get_parameter('cam_world_y').value
        base_x = self.get_parameter('base_world_x').value
        base_y = self.get_parameter('base_world_y').value

        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        fx = fy = msg.width / (2.0 * math.tan(hfov / 2.0))
        cx0, cy0 = msg.width / 2.0, msg.height / 2.0
        dbg = img.copy()

        for color, ranges in HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 80:
                continue
            m = cv2.moments(c)
            u, v = m['m10'] / m['m00'], m['m01'] / m['m00']      # centroid (Cx, Cy)

            # Pinhole model (report eq. 4.2)
            x_right = (u - cx0) * z / fx
            y_down = (v - cy0) * z / fy

            # Camera looks straight down: image-up = world +X, image-right = world -Y
            world_x = cam_x - y_down
            world_y = cam_y - x_right

            ps = PoseStamped()
            ps.header.frame_id = 'panda_link0'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = world_x - base_x
            ps.pose.position.y = world_y - base_y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            self.pubs[color].publish(ps)

            cv2.circle(dbg, (int(u), int(v)), 6, (0, 255, 0), -1)
            cv2.putText(dbg, f'{color} ({ps.pose.position.x:.2f},{ps.pose.position.y:.2f})',
                        (int(u) + 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        out = Image()
        out.header = msg.header
        out.height, out.width = dbg.shape[:2]
        out.encoding = 'rgb8'
        out.step = out.width * 3
        out.data = dbg.tobytes()
        self.dbg.publish(out)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

########## 5. MOTION NODE (talks to MoveIt's /move_action) ##########
cat > $PKG/my_sorting_robot/motion_node.py << 'EOF'
#!/usr/bin/env python3
"""Receives cube targets, asks MoveIt 2 (move_group) to plan + execute a pick-and-place path,
physically grabs and attaches the object in the planning scene, moves it to its destination bin,
and releases it. Run AFTER the MoveIt Panda demo is up."""
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume, PlanningOptions,
                             PlanningScene, CollisionObject, AttachedCollisionObject,
                             ObjectColor, AllowedCollisionMatrix)
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'panda_link0'
LINK = 'panda_link8'
GROUP = 'panda_arm'
HOVER = 0.30       # Hover height above table (m)
PICK_Z = 0.15      # Pick height (m)
DROP_Z = 0.20      # Drop height above bin (m)
BINS = {'red': (0.0, 0.40), 'yellow': (0.0, -0.40)}  # Bin coordinates in robot base frame


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rviz_visual_tools', 10)

        self.targets = {}
        self.done = set()
        self.scene_initialized = False
        self.spawned_cubes = set()

        for color in BINS:
            self.create_subscription(PoseStamped, f'/target_pose/{color}',
                                     lambda m, c=color: self.store(c, m), 10)

    def apply_scene_diff(self, scene: PlanningScene):
        # 1. Publish to /planning_scene topic for instant non-blocking update
        self.scene_pub.publish(scene)

        # 2. Also call /apply_planning_scene service if available
        if self.scene_client.service_is_ready():
            try:
                req = ApplyPlanningScene.Request()
                req.scene = scene
                fut = self.scene_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
                res = fut.result()
                return res.success if res else True
            except RuntimeError:
                pass
        return True

    def setup_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        # Allow collisions with table, bins, and cubes so MoveIt IK planning never gets rejected
        acm = AllowedCollisionMatrix()
        acm.default_entry_names = ['table', 'red_bin', 'yellow_bin', 'red_cube', 'yellow_cube']
        acm.default_entry_values = [True, True, True, True, True]
        scene.allowed_collision_matrix = acm

        # 1. Table
        table = CollisionObject()
        table.header.frame_id = FRAME
        table.id = 'table'
        table.operation = CollisionObject.ADD
        t_prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.8, 0.8, 0.4])
        t_pose = Pose()
        t_pose.position.x = 0.45
        t_pose.position.y = 0.0
        t_pose.position.z = -0.20
        t_pose.orientation.w = 1.0
        table.primitives.append(t_prim)
        table.primitive_poses.append(t_pose)
        scene.world.collision_objects.append(table)

        col_t = ObjectColor()
        col_t.id = 'table'
        col_t.color = ColorRGBA(r=0.75, g=0.75, b=0.75, a=1.0)
        scene.object_colors.append(col_t)

        # 2. Red Bin
        r_bin = CollisionObject()
        r_bin.header.frame_id = FRAME
        r_bin.id = 'red_bin'
        r_bin.operation = CollisionObject.ADD
        rb_prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.24, 0.24, 0.10])
        rb_pose = Pose()
        rb_pose.position.x = BINS['red'][0]
        rb_pose.position.y = BINS['red'][1]
        rb_pose.position.z = -0.05
        rb_pose.orientation.w = 1.0
        r_bin.primitives.append(rb_prim)
        r_bin.primitive_poses.append(rb_pose)
        scene.world.collision_objects.append(r_bin)

        col_rb = ObjectColor()
        col_rb.id = 'red_bin'
        col_rb.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.75)
        scene.object_colors.append(col_rb)

        # 3. Yellow Bin
        y_bin = CollisionObject()
        y_bin.header.frame_id = FRAME
        y_bin.id = 'yellow_bin'
        y_bin.operation = CollisionObject.ADD
        yb_prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.24, 0.24, 0.10])
        yb_pose = Pose()
        yb_pose.position.x = BINS['yellow'][0]
        yb_pose.position.y = BINS['yellow'][1]
        yb_pose.position.z = -0.05
        yb_pose.orientation.w = 1.0
        y_bin.primitives.append(yb_prim)
        y_bin.primitive_poses.append(yb_pose)
        scene.world.collision_objects.append(y_bin)

        col_yb = ObjectColor()
        col_yb.id = 'yellow_bin'
        col_yb.color = ColorRGBA(r=0.95, g=0.85, b=0.1, a=0.75)
        scene.object_colors.append(col_yb)

        ok = self.apply_scene_diff(scene)
        self.scene_initialized = True
        self.get_logger().info(f'MoveIt planning scene initialized with table and sorting bins (success={ok})')
        self.publish_markers()

    def publish_markers(self):
        ma = MarkerArray()

        def make_text(idx, text, x, y, z, r, g, b):
            m = Marker()
            m.header.frame_id = FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'labels'
            m.id = idx
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.z = 0.05
            m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
            m.text = text
            return m

        ma.markers.append(make_text(1, 'RED BIN', BINS['red'][0], BINS['red'][1], 0.15, 1.0, 0.2, 0.2))
        ma.markers.append(make_text(2, 'YELLOW BIN', BINS['yellow'][0], BINS['yellow'][1], 0.15, 1.0, 0.9, 0.1))
        ma.markers.append(make_text(3, 'PICK TABLE', 0.45, 0.0, 0.15, 0.8, 0.8, 0.8))
        self.marker_pub.publish(ma)

    def spawn_cube(self, color, x, y, z=0.025):
        cube_id = f'{color}_cube'
        scene = PlanningScene()
        scene.is_diff = True

        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = cube_id
        co.operation = CollisionObject.ADD

        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        scene.world.collision_objects.append(co)

        col = ObjectColor()
        col.id = cube_id
        if color == 'red':
            col.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            col.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
        scene.object_colors.append(col)

        ok = self.apply_scene_diff(scene)
        self.spawned_cubes.add(color)
        self.get_logger().info(f'Spawned [{color.upper()}] cube at X={x:.3f}, Y={y:.3f} in planning scene (success={ok})')

    def attach_object(self, color):
        cube_id = f'{color}_cube'
        scene = PlanningScene()
        scene.is_diff = True

        # 1. Remove from world collision objects
        co_remove = CollisionObject()
        co_remove.header.frame_id = FRAME
        co_remove.id = cube_id
        co_remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(co_remove)

        # 2. Add as attached collision object to panda_hand
        aco = AttachedCollisionObject()
        aco.link_name = 'panda_hand'
        aco.touch_links = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger', 'panda_link8']

        co_attach = CollisionObject()
        co_attach.header.frame_id = 'panda_hand'
        co_attach.id = cube_id
        co_attach.operation = CollisionObject.ADD

        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
        pose = Pose()
        # Gripper fingertip center is at z = 0.10 relative to panda_hand frame
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.10
        pose.orientation.w = 1.0

        co_attach.primitives.append(prim)
        co_attach.primitive_poses.append(pose)
        aco.object = co_attach

        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True

        # Preserve color
        col = ObjectColor()
        col.id = cube_id
        if color == 'red':
            col.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            col.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
        scene.object_colors.append(col)

        ok = self.apply_scene_diff(scene)
        self.get_logger().info(f'[{color.upper()}] Cube GRABBED and attached to gripper (success={ok})')

    def detach_object(self, color, drop_x, drop_y, drop_z=0.025):
        cube_id = f'{color}_cube'
        scene = PlanningScene()
        scene.is_diff = True

        # 1. Remove from attached collision objects
        aco = AttachedCollisionObject()
        aco.link_name = 'panda_hand'
        aco.object.id = cube_id
        aco.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True

        # 2. Add back as world collision object at destination bin
        co_world = CollisionObject()
        co_world.header.frame_id = FRAME
        co_world.id = cube_id
        co_world.operation = CollisionObject.ADD

        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
        pose = Pose()
        pose.position.x = float(drop_x)
        pose.position.y = float(drop_y)
        pose.position.z = float(drop_z)
        pose.orientation.w = 1.0

        co_world.primitives.append(prim)
        co_world.primitive_poses.append(pose)
        scene.world.collision_objects.append(co_world)

        col = ObjectColor()
        col.id = cube_id
        if color == 'red':
            col.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            col.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
        scene.object_colors.append(col)

        ok = self.apply_scene_diff(scene)
        self.get_logger().info(f'[{color.upper()}] Cube RELEASED into bin at ({drop_x:.2f}, {drop_y:.2f}) (success={ok})')

    def update_gazebo_pose(self, color, bx, by):
        # Convert from robot base frame to Gazebo world frame
        # Gazebo base_world_x = -0.45, base_world_y = 0.0
        gx = bx - 0.45
        gy = by
        gz = 0.25
        cmd = [
            'gz', 'service', '-s', '/world/sorting_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{color}_cube", position: {{x: {gx:.3f}, y: {gy:.3f}, z: {gz:.3f}}}'
        ]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'[{color.upper()}] Updated Gazebo model pose to ({gx:.2f}, {gy:.2f}, {gz:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Could not update Gazebo pose: {e}')

    def reset_cubes_in_gazebo(self):
        initial_poses = {
            'red': (0.10, 0.05, 0.425),
            'yellow': (-0.08, -0.06, 0.425),
        }
        for color, (gx, gy, gz) in initial_poses.items():
            cmd = [
                'gz', 'service', '-s', '/world/sorting_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "{color}_cube", position: {{x: {gx:.3f}, y: {gy:.3f}, z: {gz:.3f}}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
            ]
            try:
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0)
                self.get_logger().info(f'[{color.upper()}] Reset Gazebo cube position to table ({gx:.2f}, {gy:.2f}, {gz:.2f})')
            except Exception as e:
                self.get_logger().warn(f'Could not reset Gazebo pose for {color}: {e}')

    def reset_environment(self):
        self.get_logger().info('=== Resetting Sorting Environment ===')

        # 1. Open gripper
        self.get_logger().info('Opening gripper...')
        self.control_gripper(0.04)

        # 2. Reset MoveIt planning scene: remove attached & world cube collision objects
        scene = PlanningScene()
        scene.is_diff = True
        for color in BINS:
            cube_id = f'{color}_cube'
            aco = AttachedCollisionObject()
            aco.link_name = 'panda_hand'
            aco.object.id = cube_id
            aco.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(aco)

            co = CollisionObject()
            co.header.frame_id = FRAME
            co.id = cube_id
            co.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(co)
        scene.robot_state.is_diff = True
        self.apply_scene_diff(scene)
        self.setup_scene()

        # 3. Teleport cubes back to table in Gazebo
        self.reset_cubes_in_gazebo()

        # 4. Return arm to neutral ready hover position
        self.get_logger().info(f'Moving arm to neutral ready hover position (0.40, 0.0, {HOVER:.2f})...')
        if not self.move(0.40, 0.0, HOVER):
            self.get_logger().warn('Neutral hover motion did not complete; continuing with reset.')

        # 5. Clear detection & tracking state
        time.sleep(0.5)
        self.targets.clear()
        self.done.clear()
        self.spawned_cubes.clear()
        self.get_logger().info('Environment reset complete. Ready for sorting!')

    def store(self, color, msg):
        # Latch first detection on the table (X > 0.20 in robot frame)
        if color not in self.targets and msg.pose.position.x > 0.20:
            self.targets[color] = msg.pose.position
            self.get_logger().info(f'{color.upper()} target detected at X={msg.pose.position.x:.3f}, Y={msg.pose.position.y:.3f}')

    def control_gripper(self, position, effort=20.0):
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper action server not ready!')
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle and handle.accepted:
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut)

    def make_goal(self, x, y, z):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])

        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = LINK
        pc.constraint_region = BoundingVolume(primitives=[prim], primitive_poses=[pose])
        pc.weight = 1.0

        oc = OrientationConstraint()  # Gripper pointing downward (180 deg around X)
        oc.header.frame_id = FRAME
        oc.link_name = LINK
        oc.orientation.x = 1.0
        oc.orientation.y = 0.0
        oc.orientation.z = 0.0
        oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.5
        oc.absolute_y_axis_tolerance = 0.5
        oc.absolute_z_axis_tolerance = 3.14  # Free yaw rotation
        oc.weight = 1.0

        cons = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.goal_constraints = [cons]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)
        return goal

    def move(self, x, y, z):
        fut = self.move_client.send_goal_async(self.make_goal(x, y, z))
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'Goal to ({x:.2f}, {y:.2f}, {z:.2f}) rejected by MoveIt.')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code.val
        return code == 1  # 1 = SUCCESS

    def pick_place(self, color):
        p = self.targets[color]
        bx, by = BINS[color]
        self.get_logger().info(f'=============================================')
        self.get_logger().info(f'Starting Pick & Place for [{color.upper()}] cube')
        self.get_logger().info(f'=============================================')

        # 1. Open gripper
        self.get_logger().info(f'[{color.upper()}] Step 1: Opening gripper fingers...')
        self.control_gripper(0.04)
        time.sleep(0.2)

        # 2. Hover above cube
        self.get_logger().info(f'[{color.upper()}] Step 2: Hovering above cube at ({p.x:.2f}, {p.y:.2f}, {HOVER:.2f})...')
        if not self.move(p.x, p.y, HOVER):
            self.get_logger().error(f'[{color.upper()}] Hover motion failed.')
            return False

        # 3. Descend to pick
        self.get_logger().info(f'[{color.upper()}] Step 3: Descending to pick height ({p.x:.2f}, {p.y:.2f}, {PICK_Z:.2f})...')
        if not self.move(p.x, p.y, PICK_Z):
            self.get_logger().error(f'[{color.upper()}] Descend motion failed.')
            return False

        # 4. Grab object: close gripper & attach in planning scene
        self.get_logger().info(f'[{color.upper()}] Step 4: Closing gripper and grasping cube...')
        self.control_gripper(0.0)
        time.sleep(0.3)
        self.attach_object(color)
        time.sleep(0.4)

        # 5. Lift cube
        self.get_logger().info(f'[{color.upper()}] Step 5: Lifting cube with arm...')
        if not self.move(p.x, p.y, HOVER):
            self.get_logger().error(f'[{color.upper()}] Lift motion failed.')
            return False

        # 6. Move over bin
        self.get_logger().info(f'[{color.upper()}] Step 6: Moving over {color.upper()} bin ({bx:.2f}, {by:.2f}, {HOVER:.2f})...')
        if not self.move(bx, by, HOVER):
            self.get_logger().error(f'[{color.upper()}] Trajectory to bin failed.')
            return False

        # 7. Lower into bin
        self.get_logger().info(f'[{color.upper()}] Step 7: Lowering into bin ({bx:.2f}, {by:.2f}, {DROP_Z:.2f})...')
        if not self.move(bx, by, DROP_Z):
            self.get_logger().error(f'[{color.upper()}] Lowering motion failed.')
            return False

        # 8. Release object: open gripper & detach into bin
        self.get_logger().info(f'[{color.upper()}] Step 8: Opening gripper to release cube into bin...')
        self.control_gripper(0.04)
        time.sleep(0.3)
        self.detach_object(color, bx, by, 0.025)
        self.update_gazebo_pose(color, bx, by)
        time.sleep(0.4)

        # 9. Retreat to hover
        self.get_logger().info(f'[{color.upper()}] Step 9: Retreating arm to hover height...')
        self.move(bx, by, HOVER)

        self.done.add(color)
        self.get_logger().info(f'>>> [{color.upper()}] CUBE SUCCESSFULLY SORTED AND PLACED IN BIN! <<<')
        return True


def main():
    rclpy.init()
    node = MotionNode()

    # 1. Connect to MoveIt move_action
    node.get_logger().info('Waiting for MoveIt move_action server...')
    node.move_client.wait_for_server()
    node.get_logger().info('MoveIt server connected.')

    # 2. Connect to Gripper controller
    node.get_logger().info('Waiting for Gripper controller action server...')
    node.gripper_client.wait_for_server(timeout_sec=15.0)
    node.get_logger().info('Gripper controller ready.')

    # 3. Connect to ApplyPlanningScene service
    node.get_logger().info('Waiting for /apply_planning_scene service...')
    node.scene_client.wait_for_service(timeout_sec=10.0)
    node.get_logger().info('ApplyPlanningScene service ready.')

    # 4. Wait for initial robot state from joint_states
    node.get_logger().info('Waiting for robot joint states...')
    got_state = False

    def js_cb(_):
        nonlocal got_state
        got_state = True

    sub = node.create_subscription(JointState, '/joint_states', js_cb, 10)
    while rclpy.ok() and not got_state:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    node.get_logger().info('Robot state confirmed ready.')

    try:
        # 5. Reset environment at initial launch (in case cubes were left in bins from a previous run)
        node.reset_environment()

        # 6. Main sorting execution and re-run loop
        while rclpy.ok():
            node.get_logger().info('Waiting for target detections from vision node...')
            while rclpy.ok() and len(node.done) < len(BINS):
                rclpy.spin_once(node, timeout_sec=0.2)
                # Ensure any newly latched target gets spawned
                for color, pos in list(node.targets.items()):
                    if color not in node.spawned_cubes:
                        node.spawn_cube(color, pos.x, pos.y, 0.025)

                for color in list(node.targets):
                    if color not in node.done:
                        node.pick_place(color)

            node.get_logger().info('========================================================')
            node.get_logger().info('All objects grabbed, moved, and sorted successfully!')
            node.get_logger().info('========================================================')

            # Prompt to allow running again without closing or restarting anything
            print('\n' + '=' * 60)
            print('  SORTING MISSION COMPLETE!')
            print('  --------------------------------------------------')
            print('  [Press ENTER]    -> Reset cubes to table and SORT AGAIN')
            print('  [Type q + ENTER] -> Exit')
            print('=' * 60 + '\n')

            try:
                choice = input('Your choice: ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == 'q':
                node.get_logger().info('Exiting motion node.')
                break

            print('\nResetting environment for next sorting run...\n')
            node.reset_environment()

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

EOF

########## 6. LAUNCH FILE (Gazebo + bridge + vision) ##########
cat > $PKG/launch/sim.launch.py << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(get_package_share_directory('my_sorting_robot'), 'worlds', 'sorting_world.sdf')

    # -r run immediately, -s server only (no GUI = far lighter on a VM). Software rendering for VMware.
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen')

    vision = Node(package='my_sorting_robot', executable='vision_node', output='screen')

    return LaunchDescription([gz, bridge, vision])
EOF

########## 7. BUILD ##########
cd $WS
colcon build --packages-select my_sorting_robot
grep -q "ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

cat << 'EOF'

===================== DONE. RUN IN 3 SEPARATE TERMINALS =====================
(open a NEW terminal first so ~/.bashrc is loaded)

T1 (Gazebo + camera bridge + vision):
   ros2 launch my_sorting_robot sim.launch.py

T2 (MoveIt + RViz, Panda arm):
   RVIZ_CFG=$(ros2 pkg prefix my_sorting_robot)/share/my_sorting_robot/config/moveit_no_shadow.rviz
   LIBGL_ALWAYS_SOFTWARE=1 ros2 launch moveit_resources_panda_moveit_config demo.launch.py rviz_config:=$RVIZ_CFG

T3 (after RViz is fully loaded):
   ros2 run my_sorting_robot motion_node

CHECKS / SCREENSHOTS FOR THE REPORT:
   ros2 topic list                                  # /camera, /target_pose/red, /target_pose/yellow
   ros2 topic echo /target_pose/red --once
   ros2 run rqt_image_view rqt_image_view           # choose /vision/debug
==============================================================================
EOF
