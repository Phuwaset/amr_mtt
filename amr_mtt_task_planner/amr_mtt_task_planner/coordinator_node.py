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
        
        self.get_logger().info("Initializing Pick and Place Coordinator Node...")

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
        # self.nav_client.wait_for_server()
        
        self.get_logger().info("Waiting for Arm controller action server...")
        self.arm_client.wait_for_server()
        
        self.get_logger().info("Waiting for Gripper controller action server...")
        self.gripper_client.wait_for_server()
        
        self.get_logger().info("All Action Servers are UP!")

    # -------------------------------------------------------------
    # 1. Navigation Flow
    # -------------------------------------------------------------
    def send_nav_goal(self, x, y, yaw_w=1.0, yaw_z=0.0):
        self.get_logger().info(f"Navigating to -> X:{x}, Y:{y}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = yaw_w
        goal_msg.pose.pose.orientation.z = yaw_z

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 Goal Rejected!')
            return False

        self.get_logger().info('Nav2 Goal Accepted. Moving...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
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
        pos = 0.0 if open_gripper else 0.7
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
    # The Master State Machine (Pick and Place)
    # -------------------------------------------------------------
    def start_pipeline(self):
        # The exact arm positions depend on the setup. 
        # Here we define some rough values in radians: [pan, lift, elbow, w1, w2, w3]
        HOME_POS = [0.0, -1.95, -1.88, 0.0, -4.73, 0.0]
        PRE_PICK_POS = [0.0, -1.5, -2.0, -1.57, -1.57, 0.0] 
        PICK_POS = [0.0, -1.3, -2.2, -1.57, -1.57, 0.0] 
        
        self.get_logger().info("=== STARTING PIPELINE ===")
        
        # 0. Ensure Gripper is open and Arm is Home
        self.send_gripper_goal(open_gripper=True)
        self.send_arm_goal(HOME_POS, 5)
        
        # 1. Wait for objects to spawn and robot to settle
        time.sleep(3)
        
        # 2. Move to Pre-Pick pose
        self.get_logger().info("--- Phase: Reaching ---")
        self.send_arm_goal(PRE_PICK_POS, 4)
        time.sleep(1)
        
        # 3. Move down to Pick pose
        self.send_arm_goal(PICK_POS, 3)
        time.sleep(1)
        
        # 4. Close Gripper (Grasp)
        self.get_logger().info("--- Phase: Grasping ---")
        self.send_gripper_goal(open_gripper=False)
        time.sleep(1)
        
        # 5. Move back to Pre-Pick pose (Lift)
        self.get_logger().info("--- Phase: Lifting ---")
        self.send_arm_goal(PRE_PICK_POS, 3)
        
        # 6. Move back Home
        self.send_arm_goal(HOME_POS, 4)
        
        # 7. (Optional) In full version, here we would self.send_nav_goal(...) to Point B
        
        self.get_logger().info("=== PIPELINE COMPLETED ===")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    
    # We delay execution for a few seconds to let simulation settle
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
