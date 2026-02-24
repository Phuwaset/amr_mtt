#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

# Nav2 Actions
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point

# MoveIt 2 Actions / Controllers
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ** NEW ** TF Libraries for tracking position
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CoordinatorNodeTF(Node):
    def __init__(self):
        super().__init__('coordinator_node_tf')
        
        self.get_logger().info("Initializing Smart Coordinator Node (TF Tracking)...")

        # --- TF 2 Buffer & Listener setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Action Clients ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        self.wait_for_action_servers()

    def wait_for_action_servers(self):
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        
        self.get_logger().info("Waiting for Arm action server...")
        self.arm_client.wait_for_server()
        
        self.get_logger().info("Waiting for Gripper action server...")
        self.gripper_client.wait_for_server()
        self.get_logger().info("=== ALL ACTION SERVERS ARE UP ===")

    # --------------------------------------------------------------------------
    # IK Solver Helper (Simple analytical approach for a specific case)
    # WARNING: To do full 6-DOF IK perfectly in arbitrary space, we heavily rely 
    # on MoveIt2 APIs. Here we implement a targeted 2D geometric IK mapping 
    # specifically to grab the box from the side based on distance.
    # --------------------------------------------------------------------------
    def calculate_ik_for_box_reach(self, dx_local, dy_local, dz_local):
        """
        คำนวณ 2D Inverse Kinematics (IK) แม่นยำ
        dx_local: ระยะห่างด้านหน้าหุ่น (แกนพุ่งไปข้างหน้า)
        dy_local: ระยะห่างด้านข้างซ้าย/ขวา (แกนขวาง)
        dz_local: ระยะความสูงจากฐานไหล่
        """
        # หันทิศทางแขนเข้าหาของ (Pan)
        # เนื่องจากหุ่นหันหน้าตั้งฉากกับโต๊ะ (90 องศา - Yaw 1.57) แกน Local ของ UR5
        # Pan = 0 คือ หันไปข้างหน้า (ตรงกับหน้ารถ)
        # Pan = 1.57 คือ ซ้าย, Pan = -1.57 คือ ขวา
        pan = math.atan2(dy_local, dx_local)
        
        # หาระยะกระจัดรวมในแนวราบ (รัศมี R)
        r = math.sqrt(dx_local**2 + dy_local**2)
        
        # ความยาวแขนกล UR5 (m)
        a2 = 0.425  # Upper arm
        a3 = 0.392  # Forearm
        
        # เราจะไม่เอื้อมสุดปลายแขน เพราะมีข้อต่อข้อมือต่ออีก ให้หด R เข้ามา 0.05
        # และเราอยากให้ตรงข้อศอกอยู่สูงกว่ากล่อง แล้วงุ้มข้อมือลงไปหนีบ
        r_target = r - 0.15 # ลดระยะเอื้อมลง เพื่อเผื่อความยาวของ Gripper
        z_target = dz_local + 0.25 # ให้ปลายแขนลอยอยู่เหนือกล่อง 25cm ก่อน
        
        # ป้องกัน Law of Cosines Error (เอื้อมไกลกว่าความยาวแขน)
        max_reach = a2 + a3 - 0.01
        actual_reach = math.sqrt(r_target**2 + z_target**2)
        if actual_reach > max_reach:
            self.get_logger().warn(f"Target is too far ({actual_reach:.2f}m > {max_reach:.2f}m)! Scaling distance.")
            scale = max_reach / actual_reach
            r_target *= scale
            z_target *= scale
            
        # Law of Cosines untukหามุมข้อศอก (Elbow - q3)
        D = (r_target**2 + z_target**2 - a2**2 - a3**2) / (2 * a2 * a3)
        
        # เราต้องการให้ง้อข้อศอกแบบลงมาข้างล่างเสมอ (+q3)
        try:
            q3 = math.acos(D) 
        except ValueError:
            q3 = 1.0 # fallback
            
        # หา มุมหัวไหล่ (Lift - q2)
        # พื้นฐาน UR5 - Lift = 0 แบบนอนตรง, -1.57 คือตั้งตรงขึ้นฟ้า ชี้ไปข้างหน้าต้องติดลบ
        q2_base = math.atan2(z_target, r_target)
        q2_offset = math.atan2((a3 * math.sin(q3)), (a2 + a3 * math.cos(q3)))
        q2 = q2_base - q2_offset
        # ปกติ Lift UR5 ทิ่มลงเป็นบวก เงยเป็นลบ เราต้องชดเชย
        q2 = -abs(q2) 

        # คำนวณข้อมือ (Wrist 1 - q4) เพื่อให้ปลายมือทิ่มลงพื้นเพื่อหนีบกล่อง
        # เราอยากให้ข้อมือ (Joint 4) หักลงตั้งฉากกับพื้นโลก (-90 องศา หรือ -1.57 Rad)
        # สูตร: q2 + q3 + q4 = -1.57  (ผลรวมมุมทุกข้องต้องได้ทิ่มลงดิน)
        q4 = -1.57 - (q2 + q3)
        
        # แก้ Wrist 2 และ Writst 3 ให้ Gripper อ้าตรงเป๊ะ ไม่บิดเบี้ยว
        w2 = -1.57 
        w3 = 0.0
        
        return [pan, q2, q3, q4, w2, w3]

    # -------------------------------------------------------------
    # Arm Pipeline Wrapper
    # -------------------------------------------------------------
    def send_arm_goal(self, positions_list, time_from_start_sec):
        self.get_logger().info(f"Moving Arm to: {positions_list}")
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint', 'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = positions_list
        point.time_from_start.sec = time_from_start_sec
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm Goal Rejected!')
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def send_gripper_goal(self, open_gripper=True):
        pos = 0.00 if open_gripper else 0.8
        self.get_logger().info(f"{'Opening' if open_gripper else 'Closing'} Gripper...")
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = pos
        goal_msg.command.max_effort = 100.0
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper Goal Rejected!')
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    # -------------------------------------------------------------
    # Intelligent Master State Machine
    # -------------------------------------------------------------
    def start_pipeline(self):
        HOME_POS = [0.0, -1.95, -1.88, 0.0, -4.73, 0.0]
        PLACE_ON_CHASSIS_POS = [0.0, -1.3, -2.0, -1.57, -1.57, 0.0]
        
        self.get_logger().info("\n=== MISSION START: SMART COURIER ===\n")
        self.send_gripper_goal(open_gripper=True)
        self.send_arm_goal(HOME_POS, 2)
        
        target_box_x = -1.5
        target_box_y = -0.7 # ตำแหน่งกล่องที่เรารู้จุดเกิด
        
        # 1. พยายามหาพิกัดรถเทียบกับกล่อง
        self.get_logger().info("[MISSION 1] Attempting to Calculate Distance to Target...")
        time.sleep(1) # รอ TF buffer อัพเดต
        
        distance_to_box_y = 0.6 # default safety fallback
        try:
            try:
                # ลองอ่านจาก Map
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except LookupException:
                # ถ้ายังไม่ได้ Set 2D Pose Estimate (Map ยังไม่มา) ให้ใช้ Odom เป็นทางสำรอง
                self.get_logger().warn("Map frame not found! Falling back to 'odom' frame.")
                transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # คำนวณระยะห่าง
            distance_to_box_x = target_box_x - robot_x
            distance_to_box_y = target_box_y - robot_y
            
            self.get_logger().info(f"Target Box is at X: {target_box_x}, Y: {target_box_y}")
            self.get_logger().info(f"Robot is currently at X: {robot_x:.2f}, Y: {robot_y:.2f}")
            self.get_logger().info(f"Relative Distance -> dX: {distance_to_box_x:.2f} m, dY: {distance_to_box_y:.2f} m")
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF Lookup Failed: {e}. Using Default Approach Range.")
            
            
        # =========================================================
        # ปรับตัวแปรข้อต่อแขน (IK Simulation) ให้สอดคล้องกับระยะกระจัด (Distance) 
        # =========================================================
        self.get_logger().info("[MISSION 2] Calculating Exact Inverse Kinematics (IK) for Target Pos...")
        
        shoulder_z = 0.97 # ความสูงฐานแขนจากพื้น
        box_z = 0.88      # ความสูงกล่อง (0.85 + ยกขึ้นนิดนึง)
        
        # ชดเชยระยะจากฐานรถยนต์ ไปหาหัวไหล่ (Shoulder)
        # (ไหล่อยู่ห่างจากกลางรถไปข้างหน้า 0.2m)
        shoulder_x_world = robot_x 
        shoulder_y_world = robot_y + 0.2  # เดาจากรูป รถหันหน้าไปทางแกน +Y เพราะเห็นพิกัดโลก x=-1.5, y=-1.3 หันไปหา y=-0.7
        
        # แปลงพิกัดโลก เป็นพิกัด Local (หน้ารถเป็นหลัก)
        # ถ้ารถหันหน้าไปทางเหนือ (+Y) -> ด้านหน้าคือแกน +Y โลก
        dx_local = target_box_y - shoulder_y_world # แกนเดินหน้าของรถ
        dy_local = -(target_box_x - shoulder_x_world) # ติดลบชดเชยเพื่อให้แกนซ้ายขวาตรง
        dz_shoulder = box_z - shoulder_z
        
        self.get_logger().info(f"Local Distance to Box -> Front (dX): {dx_local:.2f}m, Side (dY): {dy_local:.2f}m, Height (dZ): {dz_shoulder:.2f}m")

        # ท่าเตรียม (ลอยสูง 20cm เหนือกล่อง ตามสูตร IK)
        calculated_pre_pick = self.calculate_ik_for_box_reach(dx_local, dy_local, dz_shoulder)
        
        # ท่างับ (กดลงไปอีก 15cm ให้ใกล้กล่อง หักคอลงเพิ่ม)
        calculated_pick = self.calculate_ik_for_box_reach(dx_local, dy_local, dz_shoulder - 0.15)
        
        self.get_logger().info("[MISSION 3] Reaching for Object intelligently...")
        self.send_arm_goal(calculated_pre_pick, 3)
        self.send_arm_goal(calculated_pick, 1)
        
        self.get_logger().info("[MISSION 4] Grasping...")
        self.send_gripper_goal(open_gripper=False)
        time.sleep(1.0)
        
        self.get_logger().info("[MISSION 5] Object retrieved, placing on AMR.")
        self.send_arm_goal(calculated_pre_pick, 1) # ยกพ้นขอบโต๊ะ
        self.send_arm_goal(PLACE_ON_CHASSIS_POS, 3)
        self.send_gripper_goal(open_gripper=True)
        time.sleep(0.5)
        
        self.send_arm_goal(HOME_POS, 2)
        self.get_logger().info("=== OPERATION SUCCESSFUL ===")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNodeTF()
    try:
        node.start_pipeline()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
