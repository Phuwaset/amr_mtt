#!/usr/bin/env python3

"""
Auto Docking Package

This package contains modules for the enhanced auto-docking system.

Modules:
    - pid_controller: PID control and filtering utilities
    - docking_metrics: Performance metrics tracking
    - enhanced_docking_node: Main docking node (ROS 2)

Author: AMR-MTT Project
Version: 2.0
"""

from .pid_controller import PIDController, LowPassFilter, normalize_angle
from .docking_metrics import DockingMetrics

__all__ = [
    'PIDController',
    'LowPassFilter',
    'normalize_angle',
    'DockingMetrics',
]

__version__ = '2.0.0'
