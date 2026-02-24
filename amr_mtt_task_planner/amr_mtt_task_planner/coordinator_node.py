#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Nav2 Actions
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# MoveIt 2 Actions / Controllers
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')
        
        self.get_logger().info("Initializing Full Mobile Manipulation Coordinator Node...")

        # --- Action Clients ---
        # 1. Nav2 Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 2. Arm Client (UR5)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory')

        # 3. Gripper Client (Robotiq)
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        # Ensure servers are up
        self.wait_for_action_servers()

    def wait_for_action_servers(self):
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        
        self.get_logger().info("Waiting for Arm controller action server...")
        self.arm_client.wait_for_server()
        
        self.get_logger().info("Waiting for Gripper controller action server...")
        self.gripper_client.wait_for_server()
        
        self.get_logger().info("=== ALL ACTION SERVERS ARE UP ===")

    # -------------------------------------------------------------
    # 1. Navigation Flow
    # -------------------------------------------------------------
    def send_nav_goal(self, x, y, yaw_w, yaw_z):
        self.get_logger().info(f"Navigating to -> X: {x}, Y: {y}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Quaternion for Yaw
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = yaw_z
        goal_msg.pose.pose.orientation.w = yaw_w

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 Goal Rejected!')
            return False

        self.get_logger().info('Nav2 Goal Accepted. Moving...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Nav2 Reached Goal!')
        return True

    # -------------------------------------------------------------
    # 2. Arm Flow (Hardcoded Joint Positions for Demo)
    # -------------------------------------------------------------
    def send_arm_goal(self, positions_list, time_from_start_sec):
        self.get_logger().info(f"Moving Arm to: {positions_list}")
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint',
            'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint'
        ]
        
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

    # -------------------------------------------------------------
    # 3. Gripper Flow
    # -------------------------------------------------------------
    def send_gripper_goal(self, open_gripper=True):
        # Use 0.01 to 0.65 to avoid Gazebo physics mechanical limit stalls at 0.000
        pos = 0.01 if open_gripper else 0.65
        action_word = "Opening" if open_gripper else "Closing"
        self.get_logger().info(f"{action_word} Gripper...")
        
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
    # The Master State Machine (Nav2 + Pick and Place)
    # -------------------------------------------------------------
    def start_pipeline(self):
        # 1. Arm Positions (Radians)
        HOME_POS = [0.0, -1.95, -1.88, 0.0, -4.73, 0.0]
        
        # We need to reach to the right side (-Y direction).
        # Pan joint at -1.57 rad (-90 deg) points the arm to the right.
        PRE_PICK_POS_SIDE = [-1.57, -1.6, -1.8, -1.57, -1.57, 0.0] 
        PICK_POS_SIDE = [-1.57, -1.4, -2.1, -1.57, -1.57, 0.0] 
        
        # After picking, we swing to a resting posture over the chassis
        PLACE_ON_CHASSIS_POS = [0.0, -1.3, -2.0, -1.57, -1.57, 0.0]
        
        self.get_logger().info("\n\n=== MISSION START: Autonomous Courier ===\n")
        
        # STEP 0: Reset Arm and Gripper
        self.get_logger().info("[MISSION 0/5] Resetting Posture...")
        self.send_gripper_goal(open_gripper=True)
        self.send_arm_goal(HOME_POS, 2)
        
        # STEP 1: (SKIPPED in Phase 1) Navigate to the Table
        self.get_logger().info("\n[MISSION 1/5] (SKIPPED) Robot is already stationary beside the table...")
        # self.send_nav_goal(x=-0.6, y=0.0, yaw_w=0.0, yaw_z=1.0)
        
        # STEP 2: Pre-Grasp (Reach for the box)
        self.get_logger().info("\n[MISSION 2/5] Reaching for the Box (Left side)...")
        self.send_arm_goal(PRE_PICK_POS_SIDE, 3)
        self.send_arm_goal(PICK_POS_SIDE, 1)
        
        # STEP 3: Grasp
        self.get_logger().info("\n[MISSION 3/5] Grasping the Box...")
        self.send_gripper_goal(open_gripper=False)
        time.sleep(0.5)
        
        # STEP 4: Lift
        self.get_logger().info("\n[MISSION 4/5] Lifting the Box...")
        self.send_arm_goal(PRE_PICK_POS_SIDE, 1)
        
        # STEP 5: Place on Chassis
        self.get_logger().info("\n[MISSION 5/5] Placing Box on AMR Chassis...")
        self.send_arm_goal(PLACE_ON_CHASSIS_POS, 3)
        self.send_gripper_goal(open_gripper=True)
        time.sleep(0.5)
        
        self.get_logger().info("\n[MISSION 6/6] Returning to Home...")
        self.send_arm_goal(HOME_POS, 2)
        
        self.get_logger().info("\n\n=== MISSION COMPLETE! ===\n")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    
    # Delay execution to let everything settle
    time.sleep(2) 
    
    try:
        node.start_pipeline()
    except KeyboardInterrupt:
        node.get_logger().info("Pipeline Interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
