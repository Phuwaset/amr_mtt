#!/usr/bin/env python3

"""
Docking Quality Metrics Tracker

This module provides the DockingMetrics class for tracking
and analyzing the quality and performance of docking operations.

Metrics tracked:
- Number of attempts
- Total duration
- Position accuracy (mm)
- Orientation accuracy (degrees)
- Success/failure status

Author: AMR-MTT Project
Version: 2.0
"""

import time
from typing import Dict, Optional


class DockingMetrics:
    """
    Track docking quality metrics and performance statistics.
    
    This class monitors complete docking cycles from initiation
    to completion, collecting data for quality analysis and
    system optimization.
    
    Attributes:
        start_time (float): Unix timestamp when docking started
        end_time (float): Unix timestamp when docking completed
        attempts (int): Number of docking attempts made
        position_error_mm (float): Final position error in millimeters
        orientation_error_deg (float): Final orientation error in degrees
        success (bool): Whether docking was successful
    
    Example:
        >>> metrics = DockingMetrics()
        >>> metrics.start_docking()
        >>> # ... perform docking ...
        >>> metrics.complete_docking(success=True, position_error=5.2)
        >>> summary = metrics.get_summary()
    """
    
    def __init__(self):
        """Initialize metrics tracker with default values."""
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.attempts: int = 0
        self.position_error_mm: float = 0.0
        self.orientation_error_deg: float = 0.0
        self.success: bool = False
    
    def start_docking(self) -> None:
        """
        Mark the start of a new docking attempt.
        
        This increments the attempt counter and records the start time.
        Should be called when docking sequence begins.
        """
        self.start_time = time.time()
        self.attempts += 1
    
    def complete_docking(self, 
                        success: bool, 
                        position_error: float = 0.0, 
                        orientation_error: float = 0.0) -> None:
        """
        Mark the completion of a docking attempt.
        
        Args:
            success (bool): Whether docking was successful
            position_error (float): Final position error in mm (default: 0.0)
            orientation_error (float): Final orientation error in degrees (default: 0.0)
        """
        self.end_time = time.time()
        self.success = success
        self.position_error_mm = position_error
        self.orientation_error_deg = orientation_error
    
    def get_duration(self) -> float:
        """
        Calculate total duration of docking operation.
        
        Returns:
            float: Duration in seconds, or 0.0 if not completed
        """
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        return 0.0
    
    def get_summary(self) -> Dict[str, any]:
        """
        Get comprehensive summary of docking metrics.
        
        Returns:
            dict: Dictionary containing all metrics:
                - attempts: Number of attempts made
                - duration_sec: Total time taken (seconds)
                - position_error_mm: Position accuracy (mm)
                - orientation_error_deg: Orientation accuracy (degrees)
                - success: Success status
        """
        return {
            'attempts': self.attempts,
            'duration_sec': self.get_duration(),
            'position_error_mm': self.position_error_mm,
            'orientation_error_deg': self.orientation_error_deg,
            'success': self.success
        }
    
    def reset(self) -> None:
        """Reset all metrics to initial state."""
        self.start_time = None
        self.end_time = None
        self.attempts = 0
        self.position_error_mm = 0.0
        self.orientation_error_deg = 0.0
        self.success = False
    
    def __str__(self) -> str:
        """String representation of metrics for logging."""
        summary = self.get_summary()
        return (
            f"DockingMetrics("
            f"attempts={summary['attempts']}, "
            f"duration={summary['duration_sec']:.2f}s, "
            f"pos_err={summary['position_error_mm']:.1f}mm, "
            f"orient_err={summary['orientation_error_deg']:.1f}°, "
            f"success={summary['success']})"
        )
