#!/usr/bin/env python3

"""
PID Controller and Filter Utilities for Auto Docking System

This module provides control components:
- PIDController: Standard PID controller with anti-windup protection
- LowPassFilter: Exponential moving average filter for smoothing
- normalize_angle: Utility function for angle normalization

Author: AMR-MTT Project
Version: 2.0
"""

import numpy as np
import math


class PIDController:
    """
    Simple PID controller with anti-windup protection.
    
    The controller implements the classic PID algorithm:
        output = Kp*error + Ki*integral(error) + Kd*derivative(error)
    
    Features:
    - Anti-windup: Prevents integral term from growing unbounded
    - Output limiting: Clamps output to specified range
    - Reset capability: Allows resetting internal state
    
    Args:
        kp (float): Proportional gain
        ki (float): Integral gain
        kd (float): Derivative gain
        output_limit (float, optional): Maximum absolute output value
    
    Example:
        >>> pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_limit=1.0)
        >>> output = pid.update(error=0.5, dt=0.02)
    """
    
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
    
    def reset(self) -> None:
        """Reset controller internal state (integral and previous error)."""
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, error: float, dt: float) -> float:
        """
        Update PID controller with new error measurement.
        
        Args:
            error (float): Current error (setpoint - measurement)
            dt (float): Time step since last update (seconds)
        
        Returns:
            float: Control output
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        if self.output_limit:
            # Clamp integral to prevent windup
            max_integral = self.output_limit / self.ki if self.ki != 0 else np.inf
            self.integral = np.clip(self.integral, -max_integral, max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        self.prev_error = error
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if self.output_limit:
            output = np.clip(output, -self.output_limit, self.output_limit)
        
        return output


class LowPassFilter:
    """
    Simple exponential moving average (EMA) low-pass filter.
    
    The filter implements first-order discrete-time filtering:
        y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    
    Args:
        alpha (float): Filter coefficient (0 < alpha <= 1)
                      Higher alpha = more responsive (less filtering)
                      Lower alpha = smoother (more filtering)
    
    Example:
        >>> lpf = LowPassFilter(alpha=0.3)
        >>> filtered_value = lpf.update(new_measurement)
    """
    
    def __init__(self, alpha: float = 0.3):
        if not 0 < alpha <= 1:
            raise ValueError("Alpha must be in range (0, 1]")
        
        self.alpha = alpha
        self.value = None
    
    def update(self, new_value: float) -> float:
        """
        Update filter with new value.
        
        Args:
            new_value (float): New measurement to filter
        
        Returns:
            float: Filtered output
        """
        if self.value is None:
            # First value - initialize filter
            self.value = new_value
        else:
            # Apply exponential moving average
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        
        return self.value
    
    def reset(self) -> None:
        """Reset filter state."""
        self.value = None


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.
    
    This is essential for angle error calculations to ensure
    the shortest rotation path is always chosen.
    
    Args:
        angle (float): Angle in radians
    
    Returns:
        float: Normalized angle in range [-pi, pi]
    
    Example:
        >>> normalize_angle(3.5 * math.pi)  # Returns -0.5 * pi
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
