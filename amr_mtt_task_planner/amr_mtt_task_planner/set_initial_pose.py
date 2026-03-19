#!/usr/bin/env python3
"""
set_initial_pose.py

ส่ง initial pose ให้ AMCL โดยอัตโนมัติ (ไม่ต้องกด 2D Pose Estimate ใน RViz)
ใช้ใน nav2.launch.py

Usage (ผ่าน launch):
  ดู nav2.launch.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class SetInitialPoseNode(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        # รับค่าจาก launch parameters
        self.declare_parameter('x',   -1.70)
        self.declare_parameter('y',   -1.30)
        self.declare_parameter('yaw',  0.0)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # รอสักครู่ให้ subscriber พร้อม
        self.timer = self.create_timer(1.0, self.publish_pose)
        self._published = False

    def publish_pose(self):
        if self._published:
            return

        x   = self.get_parameter('x').value
        y   = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value

        yaw_rad = math.radians(yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Covariance — ค่าปกติสำหรับ initial pose
        msg.pose.covariance[0]  = 0.25   # xx
        msg.pose.covariance[7]  = 0.25   # yy
        msg.pose.covariance[35] = 0.0685 # yaw yaw

        self.pub.publish(msg)
        self.get_logger().info(
            f'Initial pose set: x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}°'
        )
        self._published = True
        self.timer.cancel()



def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPoseNode()
    # spin จนกว่าจะ publish เสร็จ แล้วออก
    while rclpy.ok() and not node._published:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
