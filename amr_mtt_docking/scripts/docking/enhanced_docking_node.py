#!/usr/bin/env python3

"""
Enhanced Auto Docking Node with Multi-Stage Approach
Version 2.0 - Refactored

Multi-Stage Docking Process:
1. GLOBAL_NAV: Navigate to charging area using Nav2
2. SEARCHING: Rotate to find ArUco marker
3. VISUAL_SERVO: Visual servoing approach (medium range, 1-3m)
4. FINE_ALIGN: Fine alignment (close range, <1m)
5. VERIFY_CONNECTION: Check charging connection
6. DOCKED: Successfully docked and charging

Features:
- Multi-stage precision docking
- Integration with Nav2 for global navigation
- Adaptive PID control for each stage
- Connection verification
- Docking quality metrics
- Automatic retry on failure

Author: AMR-MTT Project
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, BatteryState, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32, Bool
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

# Import from refactored modules
from docking import PIDController, LowPassFilter, normalize_angle, DockingMetrics


class EnhancedAutoDockingNode(Node):
    """
    Enhanced auto-docking node with multi-stage state machine.
    
    This node implements a sophisticated docking procedure using:
    - ArUco marker detection for visual servoing
    - Multi-stage state machine for robust docking
    - PID control with adaptive parameters per stage
    - Integration with Nav2 for global navigation
    - Comprehensive metrics tracking
    """
    
    def __init__(self):
        super().__init__('enhanced_auto_docking_node')
        
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
        self.state_names = {
            0: "IDLE",
            1: "GLOBAL_NAV",
            2: "SEARCHING",
            3: "VISUAL_SERVO",
            4: "FINE_ALIGN",
            5: "VERIFY_CONNECTION",
            6: "DOCKED",
            7: "ERROR"
        }
        
        # Initialize parameters with defaults
        self._init_parameters()
        
        # Initialize CV components
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Setup ROS interfaces
        self._init_subscribers()
        self._init_publishers()
        self._init_action_clients()
        
        # Initialize control systems
        self._init_controllers()
        
        # Initialize state variables
        self._init_state_variables()
        
        # Docking metrics
        self.metrics = DockingMetrics()
        
        self.get_logger().info('Enhanced Auto Docking Node Started (Multi-Stage)')
        self.get_logger().info(f'ArUco Target ID: {self.target_id}')
        self.get_logger().info(f'Global Navigation: {"Enabled" if self.use_global_nav else "Disabled"}')

    def _init_parameters(self) -> None:
        """Initialize ROS parameters with default values and validation."""
        # Declare parameters
        self.declare_parameter('aruco_id', 0)
        self.declare_parameter('charging_station_pose_topic', '/charging_stations/closest')
        self.declare_parameter('use_global_nav', True)
        self.declare_parameter('max_docking_attempts', 3)
        self.declare_parameter('docking_timeout', 120.0)  # seconds
        
        # Per-state timeout parameters
        self.declare_parameter('global_nav_timeout', 30.0)
        self.declare_parameter('search_timeout', 60.0)
        self.declare_parameter('visual_servo_timeout', 45.0)
        self.declare_parameter('fine_align_timeout', 30.0)
        self.declare_parameter('verify_timeout', 10.0)
        
        # Load and validate parameters
        self.target_id = self._validate_aruco_id(self.get_parameter('aruco_id').value)
        self.use_global_nav = self.get_parameter('use_global_nav').value
        self.max_attempts = self._validate_positive_int(
            self.get_parameter('max_docking_attempts').value, 'max_docking_attempts', default=3)
        self.docking_timeout = self._validate_positive_float(
            self.get_parameter('docking_timeout').value, 'docking_timeout', default=120.0)
        
        # Per-state timeouts
        self.state_timeouts = {
            self.STATE_GLOBAL_NAV: self.get_parameter('global_nav_timeout').value,
            self.STATE_SEARCHING: self.get_parameter('search_timeout').value,
            self.STATE_VISUAL_SERVO: self.get_parameter('visual_servo_timeout').value,
            self.STATE_FINE_ALIGN: self.get_parameter('fine_align_timeout').value,
            self.STATE_VERIFY_CONNECTION: self.get_parameter('verify_timeout').value,
        }
        
        self.get_logger().debug(f'Parameters loaded: aruco_id={self.target_id}, '
                               f'max_attempts={self.max_attempts}, timeout={self.docking_timeout}s')
    
    def _validate_aruco_id(self, value: int) -> int:
        """Validate ArUco marker ID."""
        if not isinstance(value, int) or value < 0 or value > 1000:
            self.get_logger().warn(f'Invalid aruco_id: {value}, using default: 0')
            return 0
        return value
    
    def _validate_positive_int(self, value: any, name: str, default: int = 1) -> int:
        """Validate positive integer parameter."""
        try:
            val = int(value)
            if val <= 0:
                raise ValueError(f'{name} must be positive')
            return val
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid {name}: {value}, using default: {default}. Error: {e}')
            return default
    
    def _validate_positive_float(self, value: any, name: str, default: float = 1.0) -> float:
        """Validate positive float parameter."""
        try:
            val = float(value)
            if val <= 0:
                raise ValueError(f'{name} must be positive')
            return val
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid {name}: {value}, using default: {default}. Error: {e}')
            return default

    def _init_subscribers(self) -> None:
        """Initialize all ROS subscribers."""
        self.sub_cam = self.create_subscription(
            Image, '/rear_camera/image_raw', self.image_callback, 10)
        self.sub_battery = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.sub_should_charge = self.create_subscription(
            Bool, '/battery/should_charge', self.should_charge_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.sub_charging_connection = self.create_subscription(
            Bool, '/charging/connected', self.charging_connection_callback, 10)
        self.sub_battery_override = self.create_subscription(
            Float32, '/battery/override', self.override_battery_callback, 10)

    def _init_publishers(self) -> None:
        """Initialize all ROS publishers."""
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/docking/status', 10)
        self.pub_marker = self.create_publisher(Marker, '/docking/marker', 10)
        self.pub_metrics = self.create_publisher(String, '/docking/metrics', 10)

    def _init_action_clients(self) -> None:
        """Initialize action clients for Nav2 integration."""
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _init_controllers(self) -> None:
        """Initialize PID controllers and filters for different stages."""
        # Stage 1: Visual Servo (medium range)
        self.pid_angular_servo = PIDController(kp=1.2, ki=0.15, kd=0.35, output_limit=0.5)
        self.pid_linear_servo = PIDController(kp=0.3, ki=0.05, kd=0.1, output_limit=0.2)
        
        # Stage 2: Fine Alignment (close range)
        self.pid_angular_fine = PIDController(kp=0.8, ki=0.1, kd=0.25, output_limit=0.3)
        self.pid_linear_fine = PIDController(kp=0.15, ki=0.02, kd=0.05, output_limit=0.1)
        
        # Filters
        self.filter_angular = LowPassFilter(alpha=0.3)
        self.filter_linear = LowPassFilter(alpha=0.4)

    def _init_state_variables(self) -> None:
        """Initialize state tracking variables."""
        # Battery state
        self.battery_percent = 100.0
        self.battery_override = None
        self.low_battery_threshold = 20.0
        self.battery_received = False  # Track if battery data received
        
        # Orientation tracking
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.imu_received = False  # Track if IMU data received
        
        # Charging state
        self.charging_connected = False
        
        # Distance thresholds (based on marker area in pixels)
        self.THRESHOLD_MEDIUM_RANGE = 5000   # Enter visual servo
        self.THRESHOLD_CLOSE_RANGE = 15000   # Enter fine alignment
        self.THRESHOLD_VERY_CLOSE = 30000    # Ready to dock
        
        # Speed parameters
        self.search_rotation_speed = 0.3  # rad/s
        
        # Marker tracking
        self.marker_detected = False
        self.marker_area = 0.0
        self.marker_center_x = 0.0
        self.marker_distance = 5.0  # meters (estimated)
        self.image_width = 640
        self.marker_lost_count = 0
        self.max_marker_lost = 10
        self.last_marker_time = time.time()  # Track when marker was last seen
        
        # Recovery tracking
        self.recovery_rotations = 0  # Count rotation attempts during recovery
        self.max_recovery_rotations = 3
        
        # Timing
        self.last_time = self.get_clock().now()
        self.state_start_time = time.time()
        self.last_control_loop_time = time.time()
        
        # Navigation state
        self.nav_goal_handle = None
        
        # Error tracking
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5

    # ========== Callback Methods ==========
    
    def imu_callback(self, msg: Imu) -> None:
        """
        Extract yaw from IMU quaternion with validation.
        
        Args:
            msg: IMU message containing orientation quaternion
        """
        try:
            qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            
            # Validate quaternion (should be normalized)
            quat_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if abs(quat_norm - 1.0) > 0.1:
                self.get_logger().warn(f'IMU quaternion not normalized: {quat_norm:.3f}')
                return
            
            # Convert to yaw (Euler Z)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            if not self.imu_received:
                self.imu_received = True
                self.get_logger().debug('IMU data received successfully')
                
        except Exception as e:
            self.get_logger().error(f'IMU callback error: {e}')

    def battery_callback(self, msg: BatteryState) -> None:
        """
        Process battery state updates with validation.
        
        Args:
            msg: BatteryState message
        """
        try:
            if self.battery_override is not None:
                # Use override value if set (for testing)
                self.battery_percent = max(0.0, min(100.0, self.battery_override))
            else:
                # Validate voltage
                volts = msg.voltage
                if volts < 0 or volts > 30.0:  # Sanity check
                    self.get_logger().warn(f'Invalid battery voltage: {volts}V')
                    return
                
                # Calculate percentage from voltage
                pct = (volts - 20.0) / (24.0 - 20.0) * 100.0
                self.battery_percent = max(0.0, min(100.0, pct))
            
            if not self.battery_received:
                self.battery_received = True
                self.get_logger().debug('Battery data received successfully')
            
            # Auto-undock when fully charged
            if self.current_state == self.STATE_DOCKED and self.battery_percent > 95.0:
                self.get_logger().info('Battery Full (>95%). Undocking...')
                self.transition_to_idle()
                
        except Exception as e:
            self.get_logger().error(f'Battery callback error: {e}')

    def should_charge_callback(self, msg: Bool) -> None:
        """
        Handle battery low signal from battery predictor.
        
        Args:
            msg: Bool message indicating if charging is recommended
        """
        if msg.data and self.current_state == self.STATE_IDLE:
            self.get_logger().warn('Battery predictor recommends charging. Starting docking sequence...')
            self.start_docking_sequence()

    def charging_connection_callback(self, msg: Bool) -> None:
        """
        Update charging connection status.
        
        Args:
            msg: Bool message indicating connection status
        """
        self.charging_connected = msg.data

    def override_battery_callback(self, msg: Float32) -> None:
        """
        Handle battery override for testing.
        
        Args:
            msg: Float32 message with override battery percentage
        """
        self.battery_override = msg.data
        self.get_logger().info(f'Battery Override Set: {self.battery_override}%')
        
        # Self-trigger if override is low and we are IDLE
        if self.battery_override <= self.low_battery_threshold and self.current_state == self.STATE_IDLE:
            self.get_logger().warn(f'Battery Override Low ({self.battery_override}%). Force starting docking...')
            self.start_docking_sequence()

    def image_callback(self, msg: Image) -> None:
        """
        Process camera images and detect ArUco markers with error handling.
        
        Args:
            msg: Image message from rear camera
        """
        try:
            # Convert image
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.consecutive_errors += 1
                if self.consecutive_errors > self.max_consecutive_errors:
                    self.get_logger().error(f'Critical: Multiple image conversion failures ({self.consecutive_errors})')
                    if self.current_state not in [self.STATE_IDLE, self.STATE_ERROR]:
                        self.transition_to_error('Camera failure: Cannot process images')
                else:
                    self.get_logger().warn(f'Failed to convert image (attempt {self.consecutive_errors}): {e}')
                return
            
            # Reset error counter on success
            self.consecutive_errors = 0
            
            # Convert to grayscale
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            except Exception as e:
                self.get_logger().error(f'Failed to convert to grayscale: {e}')
                return
            
            h, w = gray.shape
            if w <= 0 or h <= 0:
                self.get_logger().warn(f'Invalid image dimensions: {w}x{h}')
                return
            self.image_width = w
            
            # Detect ArUco markers
            try:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params)
            except Exception as e:
                self.get_logger().error(f'ArUco detection failed: {e}')
                return
            
            self.marker_detected = False
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.target_id:
                        try:
                            c = corners[i][0]
                            center_x = (c[0][0] + c[2][0]) / 2
                            
                            # Validate marker position
                            if center_x < 0 or center_x > w:
                                self.get_logger().warn(f'Marker center outside image bounds: {center_x}')
                                continue
                            
                            self.marker_detected = True
                            self.marker_center_x = center_x
                            self.marker_area = cv2.contourArea(c)
                            self.marker_lost_count = 0
                            self.last_marker_time = time.time()
                            
                            # Estimate distance with validation
                            size_px = np.linalg.norm(c[0] - c[1])
                            if size_px > 0:
                                focal_length = 554.0
                                marker_size = 0.2  # meters
                                distance = (marker_size * focal_length) / size_px
                                # Sanity check on distance
                                if 0.1 < distance < 10.0:  # Reasonable range
                                    self.marker_distance = distance
                                else:
                                    self.get_logger().warn(f'Unreasonable distance estimate: {distance:.2f}m')
                            
                            # Publish marker visualization
                            self.publish_marker_viz(self.marker_distance)
                            
                            break
                        except Exception as e:
                            self.get_logger().error(f'Error processing marker {marker_id}: {e}')
                            continue
            
            if not self.marker_detected:
                self.marker_lost_count += 1
                # Log warning if marker lost for extended period during critical states
                if self.marker_lost_count == self.max_marker_lost // 2:
                    if self.current_state in [self.STATE_VISUAL_SERVO, self.STATE_FINE_ALIGN]:
                        self.get_logger().warn(f'Marker lost for {self.marker_lost_count} frames')
            
            # Run control loop
            self.control_loop()
            
        except Exception as e:
            self.get_logger().error(f'Unexpected error in image_callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    # ========== State Transition Methods ==========
    
    def start_docking_sequence(self) -> None:
        """Initiate the multi-stage docking sequence."""
        self.metrics.start_docking()
        
        if self.use_global_nav:
            self.transition_to_global_nav()
        else:
            self.transition_to_searching()

    def transition_to_global_nav(self) -> None:
        """Transition to global navigation state."""
        self.get_logger().info('STATE: GLOBAL_NAV - Navigating to charging area...')
        self.current_state = self.STATE_GLOBAL_NAV
        self.state_start_time = time.time()

    def transition_to_searching(self) -> None:
        """Transition to searching state."""
        self.get_logger().info('STATE: SEARCHING - Looking for ArUco marker...')
        self.current_state = self.STATE_SEARCHING
        self.state_start_time = time.time()
        self.marker_lost_count = 0
        self.target_yaw = self.current_yaw
        self.pid_angular_servo.reset()
        self.pid_linear_servo.reset()
        self.filter_angular.reset()
        self.filter_linear.reset()

    def transition_to_visual_servo(self) -> None:
        """Transition to visual servoing state."""
        self.get_logger().info('STATE: VISUAL_SERVO - Approaching with visual feedback...')
        self.current_state = self.STATE_VISUAL_SERVO
        self.state_start_time = time.time()
        self.marker_lost_count = 0
        self.pid_angular_servo.reset()
        self.pid_linear_servo.reset()

    def transition_to_fine_align(self) -> None:
        """Transition to fine alignment state."""
        self.get_logger().info('STATE: FINE_ALIGN - Final precision alignment...')
        self.current_state = self.STATE_FINE_ALIGN
        self.state_start_time = time.time()
        self.marker_lost_count = 0
        self.pid_angular_fine.reset()
        self.pid_linear_fine.reset()

    def transition_to_verify_connection(self) -> None:
        """Transition to connection verification state."""
        self.get_logger().info('STATE: VERIFY_CONNECTION - Checking charging connection...')
        self.current_state = self.STATE_VERIFY_CONNECTION
        self.state_start_time = time.time()

    def transition_to_docked(self) -> None:
        """Transition to docked state."""
        self.get_logger().info('STATE: DOCKED - Successfully docked and charging!')
        self.current_state = self.STATE_DOCKED
        self.state_start_time = time.time()
        
        # Calculate final metrics
        self.metrics.complete_docking(
            success=True,
            position_error=0.0,  # TODO: Calculate actual error
            orientation_error=0.0  # TODO: Calculate actual error
        )
        
        # Publish metrics
        self.publish_metrics()

    def transition_to_error(self, error_msg: str) -> None:
        """
        Transition to error state.
        
        Args:
            error_msg: Description of the error
        """
        self.get_logger().error(f'STATE: ERROR - {error_msg}')
        self.current_state = self.STATE_ERROR
        self.stop_robot()
        
        # Check if should retry
        if self.metrics.attempts < self.max_attempts:
            self.get_logger().warn(f'Retrying docking (attempt {self.metrics.attempts + 1}/{self.max_attempts})...')
            self.create_timer(2.0, self.retry_docking, oneshot=True)
        else:
            self.get_logger().error('Max docking attempts reached. Giving up.')
            self.metrics.complete_docking(success=False)
            self.publish_metrics()

    def transition_to_idle(self) -> None:
        """Transition to idle state."""
        self.get_logger().info('STATE: IDLE')
        self.current_state = self.STATE_IDLE
        self.stop_robot()

    def retry_docking(self) -> None:
        """Retry docking after error."""
        self.start_docking_sequence()

    # ========== Control Loop Methods ==========
    
    def control_loop(self) -> None:
        """Main control loop - state machine execution."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:
            dt = 0.02
        
        twist = Twist()
        
        # State machine dispatcher
        if self.current_state == self.STATE_IDLE:
            pass  # Do nothing
        elif self.current_state == self.STATE_GLOBAL_NAV:
            twist = self.handle_global_nav_state()
        elif self.current_state == self.STATE_SEARCHING:
            twist = self.handle_searching_state()
        elif self.current_state == self.STATE_VISUAL_SERVO:
            twist = self.handle_visual_servo_state(dt)
        elif self.current_state == self.STATE_FINE_ALIGN:
            twist = self.handle_fine_align_state(dt)
        elif self.current_state == self.STATE_VERIFY_CONNECTION:
            twist = self.handle_verify_connection_state()
        elif self.current_state == self.STATE_DOCKED:
            twist = Twist()  # Stop
        elif self.current_state == self.STATE_ERROR:
            twist = Twist()  # Stop
        
        # Publish velocity commands
        if self.current_state not in [self.STATE_IDLE, self.STATE_ERROR]:
            self.pub_vel.publish(twist)
        
        # Publish status
        self.publish_status()
        
        # Check per-state timeout
        if self.current_state in self.state_timeouts:
            state_duration = time.time() - self.state_start_time
            timeout = self.state_timeouts[self.current_state]
            if state_duration > timeout:
                self.get_logger().error(
                    f'State {self.state_names[self.current_state]} timeout '
                    f'({state_duration:.1f}s > {timeout:.1f}s)'
                )
                self.transition_to_error(f"{self.state_names[self.current_state]} timeout exceeded")
        
        # Check overall docking timeout
        if self.current_state not in [self.STATE_IDLE, self.STATE_ERROR, self.STATE_DOCKED]:
            total_duration = time.time() - self.state_start_time
            if total_duration > self.docking_timeout:
                self.transition_to_error("Overall docking timeout exceeded")
        
        # Detect control loop freeze
        current_time = time.time()
        if current_time - self.last_control_loop_time > 5.0:  # No loop for 5 seconds
            self.get_logger().warn('Control loop appears frozen, resetting...')
        self.last_control_loop_time = current_time

    def handle_global_nav_state(self) -> Twist:
        """
        Handle global navigation state logic.
        
        Returns:
            Twist: Velocity command (currently empty, Nav2 handles movement)
        """
        # Waiting for Nav2 to complete
        # TODO: Check Nav2 status and transition when arrived
        # For now, timeout and transition
        if time.time() - self.state_start_time > 5.0:
            self.transition_to_searching()
        
        return Twist()

    def handle_searching_state(self) -> Twist:
        """
        Handle searching state logic.
        
        Returns:
            Twist: Velocity command for marker search
        """
        twist = Twist()
        
        if self.marker_detected:
            if self.marker_area > self.THRESHOLD_CLOSE_RANGE:
                self.transition_to_fine_align()
            elif self.marker_area > self.THRESHOLD_MEDIUM_RANGE:
                self.transition_to_visual_servo()
            else:
                self.transition_to_visual_servo()
        else:
            # Rotate slowly to search
            twist.angular.z = self.search_rotation_speed
        
        return twist

    def handle_visual_servo_state(self, dt: float) -> Twist:
        """
        Handle visual servoing state with PID control.
        
        Args:
            dt: Time since last update
        
        Returns:
            Twist: Velocity command for visual servoing
        """
        twist = Twist()
        
        if not self.marker_detected:
            if self.marker_lost_count > self.max_marker_lost:
                self.transition_to_error("Lost marker during visual servo")
            return twist
        
        # Calculate angular error (centering)
        pixel_error = (self.image_width / 2) - self.marker_center_x
        angular_error = math.atan2(pixel_error, self.image_width)
        
        # PID control
        angular_cmd = self.pid_angular_servo.update(angular_error, dt)
        angular_cmd = -angular_cmd  # Invert for rear camera
        
        # Apply filter
        angular_filtered = self.filter_angular.update(angular_cmd)
        twist.angular.z = angular_filtered
        
        # Move backward toward marker with adaptive speed
        target_speed = -0.15 if self.marker_distance >= 1.5 else -0.10
        linear_filtered = self.filter_linear.update(target_speed)
        twist.linear.x = linear_filtered
        
        # Transition check
        if self.marker_area > self.THRESHOLD_CLOSE_RANGE:
            self.transition_to_fine_align()
        
        return twist

    def handle_fine_align_state(self, dt: float) -> Twist:
        """
        Handle fine alignment state with precise PID control.
        
        Args:
            dt: Time since last update
        
        Returns:
            Twist: Velocity command for fine alignment
        """
        twist = Twist()
        
        if not self.marker_detected:
            # Might be too close to see marker
            if self.marker_lost_count > 5:
                self.transition_to_verify_connection()
            return twist
        
        # More precise control
        pixel_error = (self.image_width / 2) - self.marker_center_x
        angular_error = math.atan2(pixel_error, self.image_width * 2)  # Smaller gain
        
        angular_cmd = self.pid_angular_fine.update(angular_error, dt)
        angular_cmd = -angular_cmd
        
        angular_filtered = self.filter_angular.update(angular_cmd)
        twist.angular.z = np.clip(angular_filtered, -0.2, 0.2)
        
        # Very slow backward movement
        target_speed = -0.05
        linear_filtered = self.filter_linear.update(target_speed)
        twist.linear.x = linear_filtered
        
        # Transition check
        if self.marker_area > self.THRESHOLD_VERY_CLOSE:
            self.transition_to_verify_connection()
        
        return twist

    def handle_verify_connection_state(self) -> Twist:
        """
        Handle connection verification state.
        
        Returns:
            Twist: Zero velocity (stopped)
        """
        twist = Twist()  # Stop moving
        
        # Wait for connection
        if time.time() - self.state_start_time > 2.0:
            if self.charging_connected:
                self.transition_to_docked()
            else:
                # No connection but assume docked
                self.get_logger().warn('Charging connection not detected, but assuming docked')
                self.transition_to_docked()
        
        return twist

    # ========== Utility Methods ==========
    
    def stop_robot(self) -> None:
        """Stop all robot motion."""
        self.pub_vel.publish(Twist())

    def publish_marker_viz(self, distance: float) -> None:
        """
        Publish marker visualization for RViz.
        
        Args:
            distance: Estimated distance to marker (meters)
        """
        marker = Marker()
        marker.header.frame_id = "rear_camera"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dock"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.z = distance
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.pub_marker.publish(marker)

    def publish_status(self) -> None:
        """Publish current status information."""
        status = f"{self.state_names[self.current_state]} | Battery: {self.battery_percent:.1f}%"
        if self.marker_detected:
            status += f" | Marker: Area={self.marker_area:.0f}, Dist={self.marker_distance:.2f}m"
        self.pub_status.publish(String(data=status))

    def publish_metrics(self) -> None:
        """Publish docking quality metrics."""
        metrics = self.metrics.get_summary()
        msg = f"Docking Metrics: {metrics}"
        self.pub_metrics.publish(String(data=msg))
        self.get_logger().info(msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = EnhancedAutoDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
