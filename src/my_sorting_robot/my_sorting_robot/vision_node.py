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
