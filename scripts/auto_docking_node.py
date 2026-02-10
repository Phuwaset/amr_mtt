#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PIDController:
    """Simple PID controller with anti-windup"""
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        
    def update(self, error, dt):
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        if self.output_limit:
            self.integral = np.clip(self.integral, -self.output_limit/self.ki, self.output_limit/self.ki)
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Output
        output = p_term + i_term + d_term
        
        if self.output_limit:
            output = np.clip(output, -self.output_limit, self.output_limit)
            
        return output

class LowPassFilter:
    """Simple exponential moving average filter"""
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None
        
    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        self.value = None

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class AutoDockingNode(Node):
    def __init__(self):
        super().__init__('auto_docking_node')
        
        # Multi-Stage Docking State Enum
        self.STATE_IDLE = 0
        self.STATE_GLOBAL_NAV = 1          # Navigate to charging area using Nav2
        self.STATE_SEARCHING = 2           # Rotate to find ArUco marker
        self.STATE_VISUAL_SERVO = 3        # Visual servoing approach (medium range)
        self.STATE_FINE_ALIGN = 4          # Fine alignment (close range)
        self.STATE_VERIFY_CONNECTION = 5   # Verify charging connection
        self.STATE_DOCKED = 6              # Successfully docked and charging
        self.STATE_ERROR = 7               # Error state
        
        self.current_state = self.STATE_IDLE
        
        # Parameters
        self.declare_parameter('aruco_id', 0)
        self.target_id = self.get_parameter('aruco_id').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ArUco Detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Subscribers
        self.sub_cam = self.create_subscription(
            Image,
            '/rear_camera/image_raw',
            self.image_callback,
            10)
            
        self.sub_battery = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
            
        self.sub_override = self.create_subscription(
            Float32,
            '/battery/override',
            self.override_callback,
            10)
            
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        # Publishers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/docking/status', 10)
        self.pub_marker = self.create_publisher(Marker, '/docking/marker', 10)
        
        # Variables
        self.battery_percent = 100.0
        self.override_voltage = -1.0
        self.low_battery_threshold = 20.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        
        # Control parameters
        self.pid_angular = PIDController(kp=1.0, ki=0.1, kd=0.3, output_limit=0.4)
        self.filter_angular = LowPassFilter(alpha=0.3)
        self.filter_linear = LowPassFilter(alpha=0.3)
        
        self.angular_deadzone = 0.05  # rad/s
        self.reverse_speed = -0.15  # m/s (approach)
        self.slow_reverse_speed = -0.05  # m/s (aligning)
        self.search_rotation_speed = 0.3  # rad/s
        
        # Marker tracking
        self.marker_detected = False
        self.marker_area = 0.0
        self.marker_center_x = 0.0
        self.image_width = 640
        
        # Timing
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Auto Docking Node Started (IMU-Assisted). Waiting for Low Battery...')

    def imu_callback(self, msg):
        # Extract yaw from quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Convert to yaw (Euler Z)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def override_callback(self, msg):
        self.override_voltage = msg.data

    def battery_callback(self, msg):
        volts = msg.voltage
        if self.override_voltage >= 0:
            volts = self.override_voltage
            
        pct = (volts - 20.0) / (24.0 - 20.0) * 100.0
        self.battery_percent = max(0.0, min(100.0, pct))
        
        if self.current_state == self.STATE_IDLE:
            if self.battery_percent < self.low_battery_threshold:
                self.get_logger().warn(f'Low Battery ({self.battery_percent:.1f}%). Switching to SEARCHING.')
                self.current_state = self.STATE_SEARCHING
                self.target_yaw = self.current_yaw
                self.pid_angular.reset()
                self.filter_angular.reset()
                self.filter_linear.reset()
        
        if self.current_state == self.STATE_DOCKED and self.battery_percent > 90.0:
             self.get_logger().info('Battery Full. Undocking/Idle.')
             self.current_state = self.STATE_IDLE

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        self.image_width = w
        
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        self.marker_detected = False
        
        if ids is not None:
             for i, marker_id in enumerate(ids.flatten()):
                 if marker_id == self.target_id:
                     c = corners[i][0]
                     center_x = (c[0][0] + c[2][0]) / 2
                     center_y = (c[0][1] + c[2][1]) / 2
                     
                     self.marker_detected = True
                     self.marker_center_x = center_x
                     self.marker_area = cv2.contourArea(c)
                     
                     # Calculate distance estimate
                     size_px = np.linalg.norm(c[0] - c[1])
                     focal_length = 554.0
                     dist = (0.2 * focal_length) / size_px if size_px > 0 else 5.0
                     
                     # Publish Marker for RViz
                     marker = Marker()
                     marker.header.frame_id = "rear_camera"
                     marker.header.stamp = self.get_clock().now().to_msg()
                     marker.ns = "dock"
                     marker.id = 0
                     marker.type = Marker.CUBE
                     marker.action = Marker.ADD
                     marker.pose.position.z = dist 
                     marker.pose.position.y = 0.0
                     marker.pose.position.x = 0.0
                     marker.pose.orientation.w = 1.0
                     marker.scale.x = 0.2
                     marker.scale.y = 0.2
                     marker.scale.z = 0.05
                     marker.color.a = 1.0
                     marker.color.g = 1.0
                     self.pub_marker.publish(marker)
                     
                     break
        
        self.control_loop()

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:
            dt = 0.02
        
        twist = Twist()
        status_msg = ""
        
        if self.current_state == self.STATE_IDLE:
            status_msg = "IDLE"
            
        elif self.current_state == self.STATE_SEARCHING:
            status_msg = "SEARCHING"
            if self.marker_detected:
                self.get_logger().info('Target Found! Switching to APPROACHING.')
                self.current_state = self.STATE_APPROACHING
                # Calculate target heading based on marker position
                pixel_error = (self.image_width / 2) - self.marker_center_x
                angle_offset = math.atan2(pixel_error, self.image_width)
                self.target_yaw = normalize_angle(self.current_yaw + angle_offset)
                self.pid_angular.reset()
            else:
                # Slow rotation
                twist.angular.z = self.search_rotation_speed
                
        elif self.current_state == self.STATE_APPROACHING:
            status_msg = "APPROACHING"
            if self.marker_detected:
                # Update target yaw based on marker position
                pixel_error = (self.image_width / 2) - self.marker_center_x
                angle_offset = math.atan2(pixel_error, self.image_width * 2)  # Smoother updates
                self.target_yaw = normalize_angle(self.current_yaw + angle_offset * 0.5)
                
                # PID control for heading
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                angular_cmd = self.pid_angular.update(yaw_error, dt)
                
                # Invert for rear camera (camera rotated 180°)
                angular_cmd = -angular_cmd
                
                # Apply dead zone
                if abs(angular_cmd) < self.angular_deadzone:
                    angular_cmd = 0.0
                
                # Low-pass filter
                angular_filtered = self.filter_angular.update(angular_cmd)
                
                twist.angular.z = angular_filtered
                twist.linear.x = self.reverse_speed
                
                # Transition to aligning when close
                if self.marker_area > 15000:  # Close enough
                    self.get_logger().info('Close to dock. Switching to ALIGNING.')
                    self.current_state = self.STATE_ALIGNING
            else:
                self.get_logger().warn('Lost Target during approach. Searching...')
                self.current_state = self.STATE_SEARCHING
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                
        elif self.current_state == self.STATE_ALIGNING:
            status_msg = "ALIGNING"
            if self.marker_detected:
                pixel_error = (self.image_width / 2) - self.marker_center_x
                angle_offset = math.atan2(pixel_error, self.image_width * 2)
                self.target_yaw = normalize_angle(self.current_yaw + angle_offset * 0.3)
                
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                angular_cmd = self.pid_angular.update(yaw_error, dt)
                angular_cmd = -angular_cmd
                
                if abs(angular_cmd) < self.angular_deadzone:
                    angular_cmd = 0.0
                
                angular_filtered = self.filter_angular.update(angular_cmd)
                
                # Slower speeds for final alignment
                twist.angular.z = np.clip(angular_filtered, -0.2, 0.2)
                twist.linear.x = self.slow_reverse_speed
                
                # Dock when very close
                if self.marker_area > 30000:
                    self.get_logger().info('DOCKED!')
                    self.current_state = self.STATE_DOCKED
            else:
                # Lost marker during alignment - assume we're docked (too close to see marker)
                self.get_logger().info('Lost marker during alignment. Assuming DOCKED.')
                self.current_state = self.STATE_DOCKED
                
        elif self.current_state == self.STATE_DOCKED:
            status_msg = "DOCKED"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        # Apply linear filtering
        if twist.linear.x != 0:
            twist.linear.x = self.filter_linear.update(twist.linear.x)
            
        # Publish commands
        if self.current_state != self.STATE_IDLE:
             self.pub_vel.publish(twist)
             
        self.pub_status.publish(String(data=status_msg))

def main(args=None):
    rclpy.init(args=args)
    node = AutoDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
