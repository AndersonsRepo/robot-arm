"""
Joystick to Target Pose Mapper

Converts gamepad joystick axes to target position/orientation deltas.
Integrates deltas into absolute target pose with workspace clamping.
"""
from __future__ import annotations
import numpy as np
from typing import Dict, Optional
from dataclasses import dataclass
from .gamepad_reader import GamepadState, GamepadReader


@dataclass
class TargetPose:
    """Target end-effector pose in Cartesian space."""
    position: np.ndarray  # [x, y, z] in meters
    orientation_roll: float  # Roll angle in radians (for wrist)
    timestamp: float


@dataclass
class JoystickMapping:
    """Joystick axis mapping configuration."""
    left_stick_x: str = "delta_x"      # Left stick X axis → X position delta
    left_stick_y: str = "delta_y"      # Left stick Y axis → Y position delta
    right_stick_y: str = "delta_z"     # Right stick Y axis → Z position delta
    right_stick_x: str = "wrist_roll"  # Right stick X axis → Wrist roll
    left_trigger: Optional[str] = None
    right_trigger: Optional[str] = None
    
    # Button mappings
    button_home: int = 0       # Button index for HOME
    button_park: int = 1       # Button index for PARK
    button_gripper_open: int = 2
    button_gripper_close: int = 3
    bumper_left: int = 4      # Motion scale down
    bumper_right: int = 5     # Motion scale up


class JoystickMapper:
    """Maps joystick input to target pose deltas and integrates to absolute pose."""
    
    def __init__(self, config: Dict, workspace_bounds: Optional[Dict] = None):
        """
        Initialize joystick mapper.
        
        Args:
            config: Configuration dict with gamepad settings:
                - gains: {xy_scale, z_scale, roll_scale} in m/s or rad/s per unit stick
                - mapping: axis assignments
                - deadzone: deadzone value
            workspace_bounds: Workspace limits dict with {x_min, x_max, y_min, y_max, z_min, z_max}
        """
        self.config = config
        self.workspace = workspace_bounds
        
        # Extract gains
        gains = config.get('gains', {})
        self.xy_scale = gains.get('xy_scale', 0.05)  # m/s per unit
        self.z_scale = gains.get('z_scale', 0.05)
        self.roll_scale = gains.get('roll_scale', 0.5)  # rad/s per unit
        
        # Extract mapping
        mapping = config.get('mapping', {})
        self.mapping = JoystickMapping(
            left_stick_x=mapping.get('left_stick_x', 'delta_x'),
            left_stick_y=mapping.get('left_stick_y', 'delta_y'),
            right_stick_y=mapping.get('right_stick_y', 'delta_z'),
            right_stick_x=mapping.get('right_stick_x', 'wrist_roll'),
            button_home=mapping.get('button_home', 0),
            button_park=mapping.get('button_park', 1),
            button_gripper_open=mapping.get('button_gripper_open', 2),
            button_gripper_close=mapping.get('button_gripper_close', 3),
            bumper_left=mapping.get('bumper_left', 4),
            bumper_right=mapping.get('bumper_right', 5)
        )
        
        # Motion scale (adjustable via bumpers)
        self.motion_scale = 1.0
        self.min_scale = 0.1
        self.max_scale = 2.0
        
        # Current target pose
        self.current_pose = TargetPose(
            position=np.array([0.0, 0.0, 0.3]),  # Default to workspace center
            orientation_roll=0.0,
            timestamp=0.0
        )
        
        # For evdev: map axis codes to our mapping
        # Common evdev codes: ABS_X=0, ABS_Y=1, ABS_RX=3, ABS_RY=4
        try:
            from evdev import ecodes
            self.axis_map = {
                'left_stick_x': ecodes.ABS_X,
                'left_stick_y': ecodes.ABS_Y,
                'right_stick_x': ecodes.ABS_RX,
                'right_stick_y': ecodes.ABS_RY,
            }
        except ImportError:
            # Fallback: use simple indices
            self.axis_map = {
                'left_stick_x': 0,
                'left_stick_y': 1,
                'right_stick_x': 2,
                'right_stick_y': 3,
            }
    
    def reset_pose(self, position: np.ndarray, roll: float = 0.0):
        """Reset current target pose to specified position/orientation."""
        self.current_pose.position = np.array(position)
        self.current_pose.orientation_roll = roll
    
    def update_from_gamepad(self, gamepad_state: GamepadState, dt: float) -> TargetPose:
        """
        Update target pose from gamepad state.
        
        Args:
            gamepad_state: Current gamepad state
            dt: Time step in seconds
            
        Returns:
            Updated target pose
        """
        # Update motion scale from bumpers
        if gamepad_state.buttons.get(self.mapping.bumper_left, False):
            self.motion_scale = max(self.min_scale, self.motion_scale - 0.1 * dt)
        if gamepad_state.buttons.get(self.mapping.bumper_right, False):
            self.motion_scale = min(self.max_scale, self.motion_scale + 0.1 * dt)
        
        # Get axis values (handle both evdev codes and simple indices)
        left_x = self._get_axis_value(gamepad_state, 'left_stick_x')
        left_y = self._get_axis_value(gamepad_state, 'left_stick_y')
        right_y = self._get_axis_value(gamepad_state, 'right_stick_y')
        right_x = self._get_axis_value(gamepad_state, 'right_stick_x')
        
        # Compute deltas
        delta_x = left_x * self.xy_scale * self.motion_scale * dt
        delta_y = left_y * self.xy_scale * self.motion_scale * dt
        delta_z = right_y * self.z_scale * self.motion_scale * dt
        delta_roll = right_x * self.roll_scale * self.motion_scale * dt
        
        # Integrate to absolute pose
        new_position = self.current_pose.position + np.array([delta_x, delta_y, delta_z])
        new_roll = self.current_pose.orientation_roll + delta_roll
        
        # Clamp to workspace
        if self.workspace:
            new_position = self._clamp_to_workspace(new_position)
        
        # Wrap roll to [-pi, pi]
        new_roll = np.arctan2(np.sin(new_roll), np.cos(new_roll))
        
        # Update current pose
        self.current_pose = TargetPose(
            position=new_position,
            orientation_roll=new_roll,
            timestamp=gamepad_state.timestamp
        )
        
        return self.current_pose
    
    def _get_axis_value(self, state: GamepadState, axis_name: str) -> float:
        """Get axis value from gamepad state, handling both evdev codes and indices."""
        # Try evdev code first
        if axis_name in self.axis_map:
            code = self.axis_map[axis_name]
            if code in state.axes:
                return state.axes[code]
            # Fallback: try as index
            if isinstance(code, int) and code < 10:
                return state.axes.get(code, 0.0)
        
        # Fallback: try direct index lookup
        idx_map = {'left_stick_x': 0, 'left_stick_y': 1, 'right_stick_x': 2, 'right_stick_y': 3}
        if axis_name in idx_map:
            return state.axes.get(idx_map[axis_name], 0.0)
        
        return 0.0
    
    def _clamp_to_workspace(self, position: np.ndarray) -> np.ndarray:
        """Clamp position to workspace bounds."""
        if not self.workspace:
            return position
        
        x_min = self.workspace.get('x_min', -np.inf)
        x_max = self.workspace.get('x_max', np.inf)
        y_min = self.workspace.get('y_min', -np.inf)
        y_max = self.workspace.get('y_max', np.inf)
        z_min = self.workspace.get('z_min', -np.inf)
        z_max = self.workspace.get('z_max', np.inf)
        
        return np.array([
            np.clip(position[0], x_min, x_max),
            np.clip(position[1], y_min, y_max),
            np.clip(position[2], z_min, z_max)
        ])
    
    def get_target_pose(self) -> TargetPose:
        """Get current target pose."""
        return self.current_pose
    
    def get_buttons(self, gamepad_state: GamepadState) -> Dict[str, bool]:
        """Extract button states for special actions."""
        return {
            'home': gamepad_state.buttons.get(self.mapping.button_home, False),
            'park': gamepad_state.buttons.get(self.mapping.button_park, False),
            'gripper_open': gamepad_state.buttons.get(self.mapping.button_gripper_open, False),
            'gripper_close': gamepad_state.buttons.get(self.mapping.button_gripper_close, False),
        }
    
    def get_motion_scale(self) -> float:
        """Get current motion scale factor."""
        return self.motion_scale


