#!/usr/bin/env python3

"""
Battery Predictor Node
ทำนายเวลาที่เหลือจากแบตเตอรี่และคำนวณว่าพอไปถึงเป้าหมายหรือไม่

Features:
- Battery life prediction based on usage patterns
- Distance-to-empty estimation
- Charging time prediction
- Smart charging recommendations
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Bool
import numpy as np
from collections import deque
import math
import time



class BatteryPredictor(Node):
    """
    Battery Prediction Node using regression model
    """
    
    def __init__(self):
        super().__init__('battery_predictor_node')
        
        # Parameters
        self.declare_parameter('battery_capacity_ah', 20.0)  # Amp-hours
        self.declare_parameter('nominal_voltage', 24.0)  # Volts
        self.declare_parameter('prediction_window', 60.0)  # seconds
        self.declare_parameter('min_samples', 10)  # Minimum samples for prediction
        self.declare_parameter('low_battery_threshold', 20.0)  # percent
        self.declare_parameter('critical_battery_threshold', 10.0)  # percent
        
        self.battery_capacity = self.get_parameter('battery_capacity_ah').value
        self.nominal_voltage = self.get_parameter('nominal_voltage').value
        self.prediction_window = self.get_parameter('prediction_window').value
        self.min_samples = self.get_parameter('min_samples').value
        self.low_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_threshold = self.get_parameter('critical_battery_threshold').value
        
        # Subscribers
        self.sub_battery = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        
        # Publishers
        self.pub_prediction = self.create_publisher(
            String,  # Will use custom msg later
            '/battery/prediction',
            10)
        
        self.pub_time_remaining = self.create_publisher(
            Float32,
            '/battery/time_remaining',
            10)
        
        self.pub_distance_remaining = self.create_publisher(
            Float32,
            '/battery/distance_remaining',
            10)
        
        self.pub_should_charge = self.create_publisher(
            Bool,
            '/battery/should_charge',
            10)
        
        self.pub_can_reach_goal = self.create_publisher(
            Bool,
            '/battery/can_reach_goal',
            10)
        
        # State variables
        self.current_battery_percent = 100.0
        self.current_voltage = self.nominal_voltage
        self.current_current = 0.0  # Amperes
        
        # Position tracking
        self.current_position = None
        self.goal_position = None
        self.total_distance_traveled = 0.0
        self.last_position = None
        
        # Velocity tracking
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Historical data storage
        self.battery_history = deque(maxlen=1000)  # (timestamp, percent, current)
        self.power_consumption_history = deque(maxlen=500)  # (timestamp, power_watts)
        self.distance_per_percent = deque(maxlen=100)  # meters per percent
        
        # Consumption models
        self.idle_power = 15.0  # Watts (sensors, computer)
        self.motion_power_factor = 50.0  # Watts per m/s
        self.rotation_power_factor = 20.0  # Watts per rad/s
        
        # Timing
        self.last_update_time = time.time()
        
        # Timer for periodic prediction
        self.create_timer(5.0, self.prediction_callback)  # Every 5 seconds
        
        self.get_logger().info('Battery Predictor Node Started')
        self.get_logger().info(f'Battery Capacity: {self.battery_capacity} Ah')
        self.get_logger().info(f'Nominal Voltage: {self.nominal_voltage} V')

    def battery_callback(self, msg: BatteryState):
        """Process battery state updates"""
        current_time = time.time()
        
        # Update current state
        self.current_voltage = msg.voltage
        self.current_current = msg.current if hasattr(msg, 'current') else 0.0
        
        # Calculate battery percentage
        # Simple linear model: 20V (empty) to 24V (full)
        min_voltage = 20.0
        max_voltage = 24.0
        self.current_battery_percent = ((self.current_voltage - min_voltage) / 
                                        (max_voltage - min_voltage)) * 100.0
        self.current_battery_percent = max(0.0, min(100.0, self.current_battery_percent))
        
        # Store in history
        self.battery_history.append((
            current_time,
            self.current_battery_percent,
            self.current_current
        ))
        
        # Calculate instantaneous power consumption
        power = abs(self.current_voltage * self.current_current)
        self.power_consumption_history.append((current_time, power))

    def odom_callback(self, msg: Odometry):
        """Track robot position and calculate distance traveled"""
        current_pos = msg.pose.pose.position
        
        if self.last_position is not None:
            # Calculate distance traveled
            dx = current_pos.x - self.last_position.x
            dy = current_pos.y - self.last_position.y
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance_traveled += distance
        
        self.current_position = current_pos
        self.last_position = current_pos

    def cmd_vel_callback(self, msg: Twist):
        """Track commanded velocities for power estimation"""
        self.current_linear_velocity = abs(msg.linear.x)
        self.current_angular_velocity = abs(msg.angular.z)

    def goal_callback(self, msg: PoseStamped):
        """Receive navigation goal and calculate distance"""
        self.goal_position = msg.pose.position
        self.get_logger().info(f'New goal received: ({self.goal_position.x:.2f}, {self.goal_position.y:.2f})')

    def estimate_power_consumption(self) -> float:
        """
        Estimate current power consumption based on motion
        Returns: Power in Watts
        """
        # Base power (idle)
        power = self.idle_power
        
        # Add motion power
        power += self.current_linear_velocity * self.motion_power_factor
        power += self.current_angular_velocity * self.rotation_power_factor
        
        return power

    def calculate_discharge_rate(self) -> float:
        """
        Calculate battery discharge rate (percent per second)
        Returns: discharge rate (%/s)
        """
        if len(self.battery_history) < self.min_samples:
            return 0.0
        
        # Use linear regression on recent data
        window_data = list(self.battery_history)[-self.min_samples:]
        
        times = np.array([d[0] for d in window_data])
        percents = np.array([d[1] for d in window_data])
        
        # Normalize time
        times = times - times[0]
        
        if len(times) < 2:
            return 0.0
        
        # Linear fit
        try:
            coeffs = np.polyfit(times, percents, 1)
            discharge_rate = abs(coeffs[0])  # percent per second
            return discharge_rate
        except:
            return 0.0

    def predict_time_remaining(self) -> float:
        """
        Predict time remaining until battery depleted (in seconds)
        Returns: seconds remaining
        """
        discharge_rate = self.calculate_discharge_rate()
        
        if discharge_rate <= 0:
            return float('inf')
        
        # Time to go from current percent to 0%
        time_remaining = self.current_battery_percent / discharge_rate
        
        return time_remaining

    def predict_distance_remaining(self) -> float:
        """
        Predict distance remaining until battery depleted (in meters)
        Returns: meters remaining
        """
        if len(self.battery_history) < 2:
            return 0.0
        
        # Calculate average distance per percent
        if self.total_distance_traveled > 0 and self.current_battery_percent < 100:
            battery_used = 100.0 - self.current_battery_percent
            if battery_used > 0:
                dist_per_percent = self.total_distance_traveled / battery_used
                self.distance_per_percent.append(dist_per_percent)
        
        if len(self.distance_per_percent) == 0:
            # Default estimation: assume 100m per percent
            return self.current_battery_percent * 100.0
        
        avg_dist_per_percent = np.mean(list(self.distance_per_percent))
        distance_remaining = self.current_battery_percent * avg_dist_per_percent
        
        return distance_remaining

    def can_reach_goal(self) -> bool:
        """
        Determine if robot can reach current goal with remaining battery
        Returns: True if goal is reachable
        """
        if self.goal_position is None or self.current_position is None:
            return True  # No goal set, assume OK
        
        # Calculate distance to goal
        dx = self.goal_position.x - self.current_position.x
        dy = self.goal_position.y - self.current_position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Estimate distance remaining
        distance_remaining = self.predict_distance_remaining()
        
        # Add safety margin (20%)
        safety_factor = 1.2
        required_distance = distance_to_goal * safety_factor
        
        can_reach = distance_remaining >= required_distance
        
        return can_reach

    def should_charge_now(self) -> bool:
        """
        Determine if robot should go to charging station
        Returns: True if should charge
        """
        # Critical battery - must charge immediately
        if self.current_battery_percent < self.critical_threshold:
            return True
        
        # Low battery - should charge if no urgent mission
        if self.current_battery_percent < self.low_threshold:
            return True
        
        # Check if can reach goal
        if self.goal_position is not None:
            if not self.can_reach_goal():
                return True
        
        return False

    def predict_charging_time(self, from_percent: float, to_percent: float) -> float:
        """
        Estimate charging time
        Args:
            from_percent: starting battery percent
            to_percent: target battery percent
        Returns: estimated charging time in seconds
        """
        # Assume constant current charging
        # Typical charging rate: 0.5C = 10A for 20Ah battery
        charging_current = 10.0  # Amperes
        
        # Calculate Ah to charge
        percent_to_charge = to_percent - from_percent
        ah_to_charge = (percent_to_charge / 100.0) * self.battery_capacity
        
        # Time = Ah / A (in hours)
        charging_time_hours = ah_to_charge / charging_current
        charging_time_seconds = charging_time_hours * 3600
        
        return charging_time_seconds

    def prediction_callback(self):
        """Periodic prediction and publishing"""
        
        # Calculate predictions
        time_remaining = self.predict_time_remaining()
        distance_remaining = self.predict_distance_remaining()
        should_charge = self.should_charge_now()
        can_reach = self.can_reach_goal()
        
        # Publish individual metrics
        self.pub_time_remaining.publish(Float32(data=time_remaining))
        self.pub_distance_remaining.publish(Float32(data=distance_remaining))
        self.pub_should_charge.publish(Bool(data=should_charge))
        self.pub_can_reach_goal.publish(Bool(data=can_reach))
        
        # Create summary message
        summary = self.create_prediction_summary(
            time_remaining, 
            distance_remaining, 
            should_charge, 
            can_reach
        )
        
        self.pub_prediction.publish(String(data=summary))
        
        # Log warnings if needed
        if should_charge:
            self.get_logger().warn(f'Battery low! Should charge. ({self.current_battery_percent:.1f}%)')
        
        if not can_reach and self.goal_position is not None:
            self.get_logger().warn('Cannot reach goal with current battery!')

    def create_prediction_summary(self, time_rem: float, dist_rem: float, 
                                  should_charge: bool, can_reach: bool) -> str:
        """Create human-readable prediction summary"""
        
        # Format time
        if time_rem == float('inf'):
            time_str = "N/A"
        elif time_rem > 3600:
            hours = time_rem / 3600
            time_str = f"{hours:.1f} hours"
        else:
            minutes = time_rem / 60
            time_str = f"{minutes:.1f} minutes"
        
        # Format distance
        if dist_rem > 1000:
            dist_str = f"{dist_rem/1000:.2f} km"
        else:
            dist_str = f"{dist_rem:.1f} m"
        
        # Create summary
        summary = f"Battery: {self.current_battery_percent:.1f}% | "
        summary += f"Time Remaining: {time_str} | "
        summary += f"Distance Remaining: {dist_str} | "
        summary += f"Should Charge: {should_charge} | "
        summary += f"Can Reach Goal: {can_reach}"
        
        return summary


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPredictor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
