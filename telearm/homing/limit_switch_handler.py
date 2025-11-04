"""
Limit Switch Handler for Homing and Safety

Manages limit switch state and homing routines for the robot arm.
Coordinates with ESP32 firmware for homing sequence execution.
"""
from __future__ import annotations
import time
import numpy as np
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass


@dataclass
class LimitSwitchState:
    """Limit switch state for all joints."""
    switches: List[bool]  # True if switch is active (tripped)
    timestamp: float
    

class LimitSwitchHandler:
    """Handler for limit switches used in homing and safety."""
    
    def __init__(self, config: Dict, num_joints: int = 5):
        """
        Initialize limit switch handler.
        
        Args:
            config: Configuration dict with limit_switches section:
                - pins: List of GPIO pins for limit switches
                - active_low: Whether switches are active low (pull-up)
                - homing_backoff_deg: Degrees to back off after homing
                - homing_speed_deg_per_sec: Speed for homing motion
            num_joints: Number of joints (default 5)
        """
        self.config = config
        self.num_joints = num_joints
        
        limit_config = config.get('limit_switches', {})
        self.pins = limit_config.get('pins', [])
        self.active_low = limit_config.get('active_low', True)
        self.homing_backoff_deg = limit_config.get('homing_backoff_deg', 5.0)
        self.homing_speed_deg_per_sec = limit_config.get('homing_speed_deg_per_sec', 30.0)
        
        # Current state (updated from ESP32 status messages)
        self.current_state = LimitSwitchState(
            switches=[False] * num_joints,
            timestamp=0.0
        )
        
        # Homing status
        self.homing_in_progress = False
        self.homing_complete = False
        self.homed_joints = [False] * num_joints
    
    def update_state(self, switch_states: List[bool], timestamp: float = None):
        """
        Update limit switch states from ESP32.
        
        Args:
            switch_states: List of boolean switch states (True = active/tripped)
            timestamp: Optional timestamp (defaults to current time)
        """
        if timestamp is None:
            timestamp = time.time()
        
        self.current_state = LimitSwitchState(
            switches=list(switch_states[:self.num_joints]),
            timestamp=timestamp
        )
    
    def is_switch_active(self, joint_index: int) -> bool:
        """Check if limit switch for a joint is active."""
        if 0 <= joint_index < len(self.current_state.switches):
            return self.current_state.switches[joint_index]
        return False
    
    def get_all_switches(self) -> List[bool]:
        """Get all limit switch states."""
        return self.current_state.switches.copy()
    
    def check_safety(self, joint_index: int, target_angle_deg: float, 
                     current_angle_deg: float, joint_limits: Dict) -> Tuple[bool, str]:
        """
        Check if target angle is safe given limit switch state.
        
        Args:
            joint_index: Joint index
            target_angle_deg: Target angle in degrees
            current_angle_deg: Current angle in degrees
            joint_limits: Dict with 'min' and 'max' keys in degrees
            
        Returns:
            (is_safe, reason) tuple
        """
        if not self.is_switch_active(joint_index):
            return True, "OK"
        
        # Switch is active - check if we're trying to move in wrong direction
        switch_tripped = self.is_switch_active(joint_index)
        
        # Determine which limit was hit based on current angle vs limits
        min_angle = joint_limits.get('min', -180.0)
        max_angle = joint_limits.get('max', 180.0)
        
        # If we're near the minimum limit and switch is active, moving toward min is unsafe
        if current_angle_deg < min_angle + 10.0 and switch_tripped:
            if target_angle_deg < current_angle_deg:
                return False, f"Joint {joint_index}: limit switch active, cannot move toward min"
        
        # If we're near the maximum limit and switch is active, moving toward max is unsafe
        if current_angle_deg > max_angle - 10.0 and switch_tripped:
            if target_angle_deg > current_angle_deg:
                return False, f"Joint {joint_index}: limit switch active, cannot move toward max"
        
        return True, "OK"
    
    def start_homing(self):
        """Mark that homing sequence has started."""
        self.homing_in_progress = True
        self.homing_complete = False
    
    def complete_homing(self, joint_index: Optional[int] = None):
        """
        Mark homing as complete.
        
        Args:
            joint_index: If specified, mark only this joint as homed.
                        If None, mark all joints as homed.
        """
        if joint_index is not None:
            if 0 <= joint_index < len(self.homed_joints):
                self.homed_joints[joint_index] = True
        else:
            self.homed_joints = [True] * self.num_joints
            self.homing_complete = True
            self.homing_in_progress = False
    
    def is_homed(self, joint_index: Optional[int] = None) -> bool:
        """
        Check if joint(s) are homed.
        
        Args:
            joint_index: If specified, check only this joint.
                        If None, check if all joints are homed.
        """
        if joint_index is not None:
            if 0 <= joint_index < len(self.homed_joints):
                return self.homed_joints[joint_index]
            return False
        
        return all(self.homed_joints)
    
    def get_homing_params(self) -> Dict:
        """Get homing parameters for ESP32 firmware."""
        return {
            'backoff_deg': self.homing_backoff_deg,
            'speed_deg_per_sec': self.homing_speed_deg_per_sec,
            'pins': self.pins,
            'active_low': self.active_low
        }
    
    def reset(self):
        """Reset homing state."""
        self.homing_in_progress = False
        self.homing_complete = False
        self.homed_joints = [False] * self.num_joints

