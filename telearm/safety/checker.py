"""
Safety Checker for Telemanipulation

Implements comprehensive safety checks for joint limits, workspace boundaries,
velocity/acceleration limits, and emergency conditions.
"""
from __future__ import annotations
from typing import Tuple, Dict, Any, Optional
import numpy as np
from ..models import ArmModel
from ..kinematics import Kinematics


class SafetyChecker:
    """Comprehensive safety checker for robot operation."""
    
    def __init__(self, robot_model: ArmModel, config: Dict[str, Any]):
        """
        Initialize safety checker.
        
        Args:
            robot_model: Robot arm model
            config: Safety configuration from teleop.yaml
        """
        self.robot_model = robot_model
        self.config = config
        self.kinematics = Kinematics(robot_model)
        
        # Safety parameters
        self.soft_limit_margin = np.deg2rad(config.get('safety', {}).get('soft_limit_margin_deg', 5.0))
        self.enforce_joint_limits = config.get('safety', {}).get('enforce_joint_limits', True)
        self.enforce_workspace = config.get('safety', {}).get('enforce_workspace', True)
        
        # Workspace boundaries
        self.workspace_bounds = self._load_workspace_bounds()
        
        # Emergency state
        self.emergency_stop = False
        self.emergency_reason = ""
        
    def _load_workspace_bounds(self) -> Optional[Dict[str, float]]:
        """Load workspace boundaries from robot config."""
        try:
            from ..models import load_robot_config
            robot_config = load_robot_config()
            workspace = robot_config.get('workspace', {})
            
            if workspace.get('type') == 'box':
                return {
                    'x_min': workspace['x_min'],
                    'x_max': workspace['x_max'],
                    'y_min': workspace['y_min'],
                    'y_max': workspace['y_max'],
                    'z_min': workspace['z_min'],
                    'z_max': workspace['z_max']
                }
        except Exception as e:
            print(f"Warning: Could not load workspace bounds: {e}")
        
        return None
    
    def check_joint_limits(self, q: np.ndarray, dq: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        Check and enforce joint limits.
        
        Args:
            q: Current joint angles
            dq: Desired joint velocities
            
        Returns:
            Tuple of (is_safe, corrected_velocities)
        """
        if not self.enforce_joint_limits:
            return True, dq
        
        corrected_dq = dq.copy()
        is_safe = True
        
        for i, (angle, vel, joint_spec) in enumerate(zip(q, dq, self.robot_model.joints)):
            min_limit = joint_spec.limit.min
            max_limit = joint_spec.limit.max
            
            # Check soft limits
            soft_min = min_limit + self.soft_limit_margin
            soft_max = max_limit - self.soft_limit_margin
            
            # Check hard limits
            if angle <= min_limit and vel < 0:
                corrected_dq[i] = 0.0
                is_safe = False
            elif angle >= max_limit and vel > 0:
                corrected_dq[i] = 0.0
                is_safe = False
            
            # Apply soft limit scaling
            elif angle < soft_min and vel < 0:
                # Reduce velocity when approaching soft limit
                scale = max(0.1, (angle - min_limit) / self.soft_limit_margin)
                corrected_dq[i] *= scale
            elif angle > soft_max and vel > 0:
                # Reduce velocity when approaching soft limit
                scale = max(0.1, (max_limit - angle) / self.soft_limit_margin)
                corrected_dq[i] *= scale
        
        return is_safe, corrected_dq
    
    def check_workspace(self, T_ee: np.ndarray) -> bool:
        """
        Check if end-effector is within workspace bounds.
        
        Args:
            T_ee: End-effector pose (4x4 matrix)
            
        Returns:
            True if within workspace
        """
        if not self.enforce_workspace or self.workspace_bounds is None:
            return True
        
        # Extract position
        pos = T_ee[:3, 3]
        
        # Check bounds
        x, y, z = pos[0], pos[1], pos[2]
        
        if (x < self.workspace_bounds['x_min'] or x > self.workspace_bounds['x_max'] or
            y < self.workspace_bounds['y_min'] or y > self.workspace_bounds['y_max'] or
            z < self.workspace_bounds['z_min'] or z > self.workspace_bounds['z_max']):
            return False
        
        return True
    
    def enforce_velocity_limits(self, dq: np.ndarray) -> np.ndarray:
        """Enforce joint velocity limits."""
        limited_dq = np.zeros_like(dq)
        
        for i, (vel, joint_spec) in enumerate(zip(dq, self.robot_model.joints)):
            max_vel = joint_spec.max_vel
            limited_dq[i] = np.clip(vel, -max_vel, max_vel)
        
        return limited_dq
    
    def enforce_acceleration_limits(self, dq: np.ndarray, dq_prev: np.ndarray, dt: float) -> np.ndarray:
        """Enforce joint acceleration limits."""
        if dt <= 0:
            return dq
        
        limited_dq = np.zeros_like(dq)
        
        for i, (vel, prev_vel, joint_spec) in enumerate(zip(dq, dq_prev, self.robot_model.joints)):
            max_acc = joint_spec.max_acc
            
            # Calculate desired acceleration
            desired_acc = (vel - prev_vel) / dt
            
            # Limit acceleration
            limited_acc = np.clip(desired_acc, -max_acc, max_acc)
            
            # Convert back to velocity
            limited_dq[i] = prev_vel + limited_acc * dt
        
        return limited_dq
    
    def check_emergency_conditions(self, q: np.ndarray, dq: np.ndarray) -> Tuple[bool, str]:
        """
        Check for emergency conditions.
        
        Args:
            q: Current joint angles
            dq: Current joint velocities
            
        Returns:
            Tuple of (is_emergency, reason)
        """
        # Check for extreme joint angles
        for i, (angle, joint_spec) in enumerate(zip(q, self.robot_model.joints)):
            if angle < joint_spec.limit.min or angle > joint_spec.limit.max:
                return True, f"Joint {i} exceeded hard limits: {np.rad2deg(angle):.1f}°"
        
        # Check for extreme velocities
        for i, (vel, joint_spec) in enumerate(zip(dq, self.robot_model.joints)):
            if abs(vel) > joint_spec.max_vel * 1.2:  # 20% over limit
                return True, f"Joint {i} velocity too high: {np.rad2deg(vel):.1f}°/s"
        
        # Check end-effector position
        T_ee = self.kinematics.fk(q)
        if not self.check_workspace(T_ee):
            return True, "End-effector outside workspace"
        
        return False, ""
    
    def comprehensive_safety_check(self, q: np.ndarray, dq: np.ndarray, 
                                  dq_prev: np.ndarray, dt: float) -> Tuple[bool, np.ndarray, str]:
        """
        Perform comprehensive safety check.
        
        Args:
            q: Current joint angles
            dq: Desired joint velocities
            dq_prev: Previous joint velocities
            dt: Time step
            
        Returns:
            Tuple of (is_safe, corrected_velocities, status_message)
        """
        status_msg = "OK"
        
        # Check emergency conditions
        is_emergency, emergency_reason = self.check_emergency_conditions(q, dq)
        if is_emergency:
            self.emergency_stop = True
            self.emergency_reason = emergency_reason
            return False, np.zeros_like(dq), f"EMERGENCY: {emergency_reason}"
        
        # Apply velocity limits
        dq_limited = self.enforce_velocity_limits(dq)
        
        # Apply acceleration limits
        dq_limited = self.enforce_acceleration_limits(dq_limited, dq_prev, dt)
        
        # Check joint limits
        is_joint_safe, dq_limited = self.check_joint_limits(q, dq_limited)
        
        if not is_joint_safe:
            status_msg = "Joint limits applied"
        
        # Check workspace
        T_ee = self.kinematics.fk(q)
        if not self.check_workspace(T_ee):
            status_msg = "Workspace violation"
            # Reduce velocities when outside workspace
            dq_limited *= 0.1
        
        return True, dq_limited, status_msg
    
    def reset_emergency(self):
        """Reset emergency state."""
        self.emergency_stop = False
        self.emergency_reason = ""
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Get current safety status."""
        return {
            'emergency_stop': self.emergency_stop,
            'emergency_reason': self.emergency_reason,
            'enforce_joint_limits': self.enforce_joint_limits,
            'enforce_workspace': self.enforce_workspace,
            'workspace_bounds': self.workspace_bounds,
            'soft_limit_margin_deg': np.rad2deg(self.soft_limit_margin)
        }


class EmergencyStopHandler:
    """Handles emergency stop conditions."""
    
    def __init__(self, config: Dict[str, Any]):
        self.emergency_stop_pin = config.get('safety', {}).get('emergency_stop_pin', 13)
        self.watchdog_timeout = config.get('timeout_ms', 200) / 1000.0  # Convert to seconds
        self.last_command_time = 0.0
        self.emergency_active = False
        
    def update_command_time(self):
        """Update last command time for watchdog."""
        import time
        self.last_command_time = time.time()
    
    def check_watchdog(self) -> bool:
        """Check if watchdog has timed out."""
        import time
        if time.time() - self.last_command_time > self.watchdog_timeout:
            self.emergency_active = True
            return False
        return True
    
    def is_emergency_active(self) -> bool:
        """Check if emergency stop is active."""
        return self.emergency_active
    
    def reset_emergency(self):
        """Reset emergency stop."""
        self.emergency_active = False
        import time
        self.last_command_time = time.time()


def test_safety_checker():
    """Test the safety checker."""
    from ..models import load_from_config
    
    print("Testing SafetyChecker...")
    
    # Load robot model
    robot_model = load_from_config()
    
    # Configuration
    config = {
        'safety': {
            'enforce_joint_limits': True,
            'enforce_workspace': True,
            'soft_limit_margin_deg': 5.0
        }
    }
    
    # Create safety checker
    checker = SafetyChecker(robot_model, config)
    
    # Test joint limits
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Home position
    dq = np.array([0.1, 0.2, 0.3, 0.1, 0.05])  # Normal velocities
    
    is_safe, corrected_dq = checker.check_joint_limits(q, dq)
    print(f"Joint limits test: safe={is_safe}, corrected_velocities={corrected_dq}")
    
    # Test extreme velocities
    extreme_dq = np.array([10.0, 10.0, 10.0, 10.0, 10.0])  # Very high velocities
    limited_dq = checker.enforce_velocity_limits(extreme_dq)
    print(f"Velocity limits test: original={extreme_dq}, limited={limited_dq}")
    
    # Test workspace
    T_ee = checker.kinematics.fk(q)
    in_workspace = checker.check_workspace(T_ee)
    print(f"Workspace test: in_workspace={in_workspace}")
    
    # Test comprehensive check
    is_safe, final_dq, status = checker.comprehensive_safety_check(q, dq, np.zeros(5), 0.01)
    print(f"Comprehensive test: safe={is_safe}, status='{status}', final_velocities={final_dq}")
    
    print("SafetyChecker test completed")
    return True


if __name__ == "__main__":
    test_safety_checker()
