"""
Velocity Integration for Telemanipulation

Integrates joint velocities to positions with safety constraints and smoothing.
"""
from __future__ import annotations
from typing import Tuple
import numpy as np
from ..models import ArmModel


class VelocityIntegrator:
    """Integrates joint velocities to positions with safety constraints."""
    
    def __init__(self, robot_model: ArmModel, dt: float = 0.01):
        """
        Initialize velocity integrator.
        
        Args:
            robot_model: Robot arm model for limits
            dt: Integration time step (seconds)
        """
        self.robot_model = robot_model
        self.dt = dt
        
        # State
        self.joint_positions = np.array([js.home for js in robot_model.joints])
        self.joint_velocities = np.zeros(robot_model.n())
        self.joint_accelerations = np.zeros(robot_model.n())
        
        # Previous state for acceleration limiting
        self.prev_velocities = np.zeros(robot_model.n())
        
        # Safety parameters
        self.max_acceleration_scale = 0.8  # 80% of max acceleration
        self.velocity_ramp_time = 0.1  # Time to ramp to full velocity
        
    def integrate(self, desired_velocities: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Integrate desired velocities to new positions.
        
        Args:
            desired_velocities: Desired joint velocities (rad/s)
            
        Returns:
            Tuple of (new_positions, actual_velocities)
        """
        # Enforce velocity limits
        limited_velocities = self._enforce_velocity_limits(desired_velocities)
        
        # Enforce acceleration limits
        limited_velocities = self._enforce_acceleration_limits(limited_velocities)
        
        # Integrate to get new positions
        new_positions = self.joint_positions + limited_velocities * self.dt
        
        # Enforce position limits
        new_positions = self._enforce_position_limits(new_positions, limited_velocities)
        
        # Update state
        self.joint_accelerations = (limited_velocities - self.joint_velocities) / self.dt
        self.prev_velocities = self.joint_velocities
        self.joint_velocities = limited_velocities
        self.joint_positions = new_positions
        
        return new_positions, limited_velocities
    
    def _enforce_velocity_limits(self, velocities: np.ndarray) -> np.ndarray:
        """Enforce joint velocity limits."""
        limited = np.zeros_like(velocities)
        
        for i, (vel, joint_spec) in enumerate(zip(velocities, self.robot_model.joints)):
            max_vel = joint_spec.max_vel
            limited[i] = np.clip(vel, -max_vel, max_vel)
        
        return limited
    
    def _enforce_acceleration_limits(self, velocities: np.ndarray) -> np.ndarray:
        """Enforce joint acceleration limits."""
        limited = np.zeros_like(velocities)
        
        for i, (vel, joint_spec, prev_vel) in enumerate(zip(velocities, self.robot_model.joints, self.prev_velocities)):
            max_acc = joint_spec.max_acc * self.max_acceleration_scale
            
            # Calculate desired acceleration
            desired_acc = (vel - prev_vel) / self.dt
            
            # Limit acceleration
            limited_acc = np.clip(desired_acc, -max_acc, max_acc)
            
            # Convert back to velocity
            limited[i] = prev_vel + limited_acc * self.dt
        
        return limited
    
    def _enforce_position_limits(self, positions: np.ndarray, velocities: np.ndarray) -> np.ndarray:
        """Enforce joint position limits with soft constraints."""
        limited = positions.copy()
        
        for i, (pos, vel, joint_spec) in enumerate(zip(positions, velocities, self.robot_model.joints)):
            min_pos = joint_spec.limit.min
            max_pos = joint_spec.limit.max
            
            # Soft limit margin (5 degrees)
            soft_margin = np.deg2rad(5.0)
            soft_min = min_pos + soft_margin
            soft_max = max_pos - soft_margin
            
            # Check soft limits
            if pos < soft_min:
                # Reduce velocity when approaching soft limit
                reduction_factor = max(0.1, (pos - min_pos) / soft_margin)
                limited[i] = self.joint_positions[i] + vel * self.dt * reduction_factor
            elif pos > soft_max:
                # Reduce velocity when approaching soft limit
                reduction_factor = max(0.1, (max_pos - pos) / soft_margin)
                limited[i] = self.joint_positions[i] + vel * self.dt * reduction_factor
            
            # Hard limits (clamp)
            limited[i] = np.clip(limited[i], min_pos, max_pos)
        
        return limited
    
    def reset_to_home(self):
        """Reset integrator to home position."""
        self.joint_positions = np.array([js.home for js in self.robot_model.joints])
        self.joint_velocities = np.zeros(self.robot_model.n())
        self.joint_accelerations = np.zeros(self.robot_model.n())
        self.prev_velocities = np.zeros(self.robot_model.n())
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get current state."""
        return (self.joint_positions.copy(), 
                self.joint_velocities.copy(), 
                self.joint_accelerations.copy())
    
    def set_positions(self, positions: np.ndarray):
        """Set joint positions (for initialization)."""
        self.joint_positions = positions.copy()
        # Reset velocities when setting positions
        self.joint_velocities = np.zeros(self.robot_model.n())
        self.joint_accelerations = np.zeros(self.robot_model.n())
        self.prev_velocities = np.zeros(self.robot_model.n())


class SmoothVelocityIntegrator(VelocityIntegrator):
    """Velocity integrator with additional smoothing."""
    
    def __init__(self, robot_model: ArmModel, dt: float = 0.01, smoothing_factor: float = 0.7):
        """
        Initialize smooth velocity integrator.
        
        Args:
            robot_model: Robot arm model
            dt: Integration time step
            smoothing_factor: Smoothing factor (0-1, higher = more smoothing)
        """
        super().__init__(robot_model, dt)
        self.smoothing_factor = smoothing_factor
        self.filtered_velocities = np.zeros(self.robot_model.n())
        
    def integrate(self, desired_velocities: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Integrate with velocity smoothing."""
        # Apply smoothing filter
        self.filtered_velocities = (self.smoothing_factor * self.filtered_velocities + 
                                   (1.0 - self.smoothing_factor) * desired_velocities)
        
        # Use filtered velocities for integration
        return super().integrate(self.filtered_velocities)


def test_velocity_integrator():
    """Test the velocity integrator."""
    from ..models import load_from_config
    
    print("Testing VelocityIntegrator...")
    
    # Load robot model
    robot_model = load_from_config()
    
    # Create integrator
    integrator = VelocityIntegrator(robot_model, dt=0.01)
    
    # Test integration
    desired_velocities = np.array([0.1, 0.2, 0.3, 0.1, 0.05])  # rad/s
    
    print(f"Initial positions: {integrator.joint_positions}")
    print(f"Desired velocities: {desired_velocities}")
    
    # Integrate for several steps
    for i in range(10):
        positions, actual_velocities = integrator.integrate(desired_velocities)
        print(f"Step {i+1}: positions={positions}, velocities={actual_velocities}")
    
    # Test position limits
    print("\nTesting position limits...")
    integrator.reset_to_home()
    
    # Try to exceed limits
    large_velocities = np.array([10.0, 10.0, 10.0, 10.0, 10.0])  # Very large velocities
    for i in range(5):
        positions, actual_velocities = integrator.integrate(large_velocities)
        print(f"Large vel step {i+1}: positions={positions}, velocities={actual_velocities}")
    
    print("VelocityIntegrator test completed")
    return True


if __name__ == "__main__":
    test_velocity_integrator()
