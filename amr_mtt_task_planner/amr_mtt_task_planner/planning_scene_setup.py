#!/usr/bin/env python3
"""
planning_scene_setup.py

Publishes โต๊ะและกล่องพัสดุเข้า MoveIt Planning Scene
เพื่อให้แสดงใน RViz และ MoveIt จะ Avoid Collision ได้ถูกต้อง

Usage:
  ros2 run amr_mtt_task_planner planning_scene_setup
"""

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class PlanningSceneSetup(Node):
    def __init__(self):
        super().__init__('planning_scene_setup')

        # Publisher ไปยัง MoveIt Planning Scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        # รอให้ MoveIt Move Group สตาร์ทก่อน
        self.get_logger().info('Waiting 3 seconds for MoveIt to be ready...')
        self.timer = self.create_timer(3.0, self.publish_scene)

    def make_box(self, name, frame_id, x, y, z, sx, sy, sz):
        """สร้าง CollisionObject รูปทรงกล่อง"""
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]  # [x_size, y_size, z_size]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def publish_scene(self):
        """Publish โต๊ะและกล่องลง Planning Scene"""

        # ============================================================
        # 1. โต๊ะ (Static Obstacle) - ตรงกับ Gazebo World
        # pick_table: center (-1.5, -0.5), top surface z=0.8
        # ============================================================
        table_top = self.make_box(
            name='pick_table_top',
            frame_id='map',
            x=-1.5, y=-0.5, z=0.785,   # ตำแหน่งกึ่งกลางหน้าโต๊ะ
            sx=0.6, sy=0.5, sz=0.03     # ขนาดเดียวกับ Gazebo
        )
        table_leg = self.make_box(
            name='pick_table_leg',
            frame_id='map',
            x=-1.5, y=-0.5, z=0.39,
            sx=0.08, sy=0.08, sz=0.78
        )
        table_base = self.make_box(
            name='pick_table_base',
            frame_id='map',
            x=-1.5, y=-0.5, z=0.015,
            sx=0.5, sy=0.4, sz=0.03
        )

        # ============================================================
        # 2. กล่องพัสดุ (Target Object) - ตรงกับ Gazebo World
        # parcel_box: center (-1.5, -0.5, 0.85)
        # ============================================================
        parcel_box = self.make_box(
            name='parcel_box',
            frame_id='map',
            x=-1.5, y=-0.5, z=0.85,
            sx=0.1, sy=0.1, sz=0.1
        )

        # Publish ทุก Object
        self.collision_pub.publish(table_top)
        self.collision_pub.publish(table_leg)
        self.collision_pub.publish(table_base)
        self.collision_pub.publish(parcel_box)

        self.get_logger().info(
            '=== Planning Scene Published ===\n'
            '  [OK] pick_table_top   @ (-1.5, -0.5, 0.785)\n'
            '  [OK] pick_table_leg   @ (-1.5, -0.5, 0.390)\n'
            '  [OK] pick_table_base  @ (-1.5, -0.5, 0.015)\n'
            '  [OK] parcel_box       @ (-1.5, -0.5, 0.850)\n'
            'Objects should now appear in RViz MotionPlanning > Scene Objects tab.'
        )
        # รัน 1 ครั้งแล้วหยุด
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneSetup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
