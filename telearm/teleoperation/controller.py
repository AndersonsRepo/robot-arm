"""
Position-Based Teleoperation Controller

Main control loop that reads gamepad, performs IK, and sends joint targets to ESP32.
Replaces velocity-based teleoperation with position-based IK control.
"""
from __future__ import annotations
import time
import threading
import yaml
import numpy as np
from typing import Dict, Any, Optional
from pathlib import Path

from ..models import load_from_config
from ..kinematics import Kinematics
from ..ik import IK, IKOptions
from ..input import GamepadReader, JoystickMapper
from ..drivers import ESP32SerialDriver
from ..safety.checker import SafetyChecker
from ..homing import LimitSwitchHandler


class PositionController:
    """Position-based control controller using gamepad input and IK."""
    
    def __init__(self, config_path: str = "config/teleop.yaml", use_sim: bool = False):
        """
        Initialize position-based controller.
        
        Args:
            config_path: Path to teleoperation configuration file
            use_sim: Use simulation mode (no hardware writes)
        """
        # Load configurations
        self.config = self._load_config(config_path)
        self.robot_config = self._load_robot_config()
        self.pins_config = self._load_pins_config()
        
        # Robot model and kinematics
        self.robot_model = load_from_config()
        self.kin = Kinematics(self.robot_model)
        
        # IK solver with configurable options
        ik_opts = IKOptions(
            max_iters=self.config['ik']['max_iters'],
            tol_pos=self.config['ik']['tol_pos'],
            tol_rot=self.config['ik']['tol_rot'],
            lambda2=self.config['ik']['damping']
        )
        self.ik = IK(self.kin)
        self.ik_opts = ik_opts
        
        # Input devices
        gamepad_config = self.config['gamepad']
        self.gamepad = GamepadReader(
            device_path=gamepad_config.get('device', 'auto'),
            deadzone=gamepad_config.get('deadzone', 0.1)
        )
        
        workspace = self.robot_config.get('workspace', {})
        self.joystick_mapper = JoystickMapper(gamepad_config, workspace)
        
        # Hardware driver
        self.use_sim = use_sim
        if not use_sim:
            esp32_config = self.pins_config['hardware']['esp32']
            self.driver = ESP32SerialDriver(
                port=esp32_config.get('default_port', '/dev/ttyUSB0'),
                baud=esp32_config.get('baud_rate', 115200),
                timeout=esp32_config.get('timeout', 0.3),
                num_joints=self.robot_model.n()
            )
        else:
            from ..drivers import NullServoDriver
            self.driver = NullServoDriver(self.robot_model.n())
        
        # Safety and homing
        self.safety = SafetyChecker(self.robot_model, self.config)
        self.limit_switches = LimitSwitchHandler(self.pins_config, self.robot_model.n())
        
        # State
        self.robot_q = np.array([js.home for js in self.robot_model.joints])
        self.target_pose = None
        self.is_running = False
        self.control_thread = None
        
        # Smoothing filter for joint targets
        self.joint_smoothing_alpha = 0.1  # Low-pass filter coefficient
        self.smoothed_q = self.robot_q.copy()
        
        # Statistics
        self.stats = {
            'ik_solves': 0,
            'ik_failures': 0,
            'safety_violations': 0,
            'packets_sent': 0,
            'start_time': None
        }
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load teleoperation configuration."""
        full_path = Path(config_path)
        if not full_path.is_absolute():
            full_path = Path(__file__).parent.parent.parent / config_path
        
        try:
            with open(full_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load config from {config_path}: {e}")
            return self._default_teleop_config()
    
    def _load_robot_config(self) -> Dict[str, Any]:
        """Load robot configuration."""
        try:
            from ..models import load_robot_config
            return load_robot_config()
        except Exception as e:
            print(f"Warning: Could not load robot config: {e}")
            return {}
    
    def _load_pins_config(self) -> Dict[str, Any]:
        """Load pins configuration."""
        config_path = Path(__file__).parent.parent.parent / "config" / "pins.yaml"
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load pins config: {e}")
            return {}
    
    def _default_teleop_config(self) -> Dict[str, Any]:
        """Return default teleoperation configuration."""
        return {
            'mode': 'gamepad',
            'update_rate_hz': 100,
            'timeout_ms': 300,
            'gamepad': {
                'device': 'auto',
                'deadzone': 0.1,
                'gains': {'xy_scale': 0.05, 'z_scale': 0.05, 'roll_scale': 0.5},
                'mapping': {}
            },
            'ik': {
                'damping': 1e-4,
                'max_iters': 200,
                'tol_pos': 1e-3,
                'tol_rot': 1e-2
            },
            'rates': {'ik_hz': 50, 'pwm_hz': 50},
            'safety': {
                'enforce_joint_limits': True,
                'enforce_workspace': True,
                'soft_limit_margin_deg': 5.0,
                'workspace_clamp': True
            }
        }
    
    def start(self):
        """Start the control loop."""
        if self.is_running:
            return
        
        print("Starting position-based controller...")
        
        # Initialize gamepad
        try:
            self.gamepad.start()
        except Exception as e:
            print(f"Failed to start gamepad: {e}")
            raise
        
        # Initialize hardware driver
        if not self.use_sim:
            if not self.driver.connect():
                print("Warning: Could not connect to ESP32, using simulation mode")
                self.use_sim = True
                from ..drivers import NullServoDriver
                self.driver = NullServoDriver(self.robot_model.n())
        
        # Get initial pose from forward kinematics
        T_init = self.kin.fk(self.robot_q)
        pos_init = T_init[:3, 3]
        self.joystick_mapper.reset_pose(pos_init, roll=0.0)
        
        # Start control loop
        self.is_running = True
        self.stats['start_time'] = time.time()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print("Position-based controller started")
    
    def stop(self):
        """Stop the control loop."""
        if not self.is_running:
            return
        
        print("Stopping position-based controller...")
        
        self.is_running = False
        
        # Stop gamepad
        self.gamepad.stop()
        
        # Disconnect driver
        if hasattr(self.driver, 'disconnect'):
            self.driver.disconnect()
        
        # Wait for control thread
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        print("Position-based controller stopped")
    
    def _control_loop(self):
        """Main control loop running at specified rate."""
        rate = self.config['update_rate_hz']
        dt = 1.0 / rate
        
        print(f"Control loop started at {rate} Hz")
        
        while self.is_running:
            loop_start = time.time()
            
            try:
                # Read gamepad state
                gamepad_state = self.gamepad.get_state()
                
                # Update target pose from joystick
                target_pose = self.joystick_mapper.update_from_gamepad(gamepad_state, dt)
                self.target_pose = target_pose
                
                # Check button commands
                buttons = self.joystick_mapper.get_buttons(gamepad_state)
                if buttons['home']:
                    self._execute_home()
                    continue
                if buttons['park']:
                    self._execute_park()
                    continue
                
                # Convert target pose to transformation matrix
                T_target = self._pose_to_transform(target_pose)
                
                # Workspace clamping (if enabled)
                if self.config['safety'].get('workspace_clamp', True):
                    T_target = self._clamp_workspace(T_target)
                
                # Solve IK
                q_target = self.ik.solve(self.robot_q, T_target, self.ik_opts)
                
                # Check IK convergence
                T_actual = self.kin.fk(q_target)
                pos_error = np.linalg.norm(T_target[:3, 3] - T_actual[:3, 3])
                if pos_error > self.ik_opts.tol_pos * 10:  # Allow some tolerance
                    self.stats['ik_failures'] += 1
                    print(f"Warning: IK did not converge well (error: {pos_error:.4f}m)")
                else:
                    self.stats['ik_solves'] += 1
                
                # Apply joint smoothing
                self.smoothed_q = (self.joint_smoothing_alpha * q_target + 
                                   (1 - self.joint_smoothing_alpha) * self.smoothed_q)
                
                # Safety checks for position-based control
                # Clamp joint angles to limits
                q_safe = self.smoothed_q.copy()
                for i, (angle, joint_spec) in enumerate(zip(q_safe, self.robot_model.joints)):
                    q_safe[i] = np.clip(angle, joint_spec.limit.min, joint_spec.limit.max)
                
                # Check workspace
                T_ee = self.kin.fk(q_safe)
                if not self.safety.check_workspace(T_ee):
                    self.stats['safety_violations'] += 1
                    print(f"Safety violation: End-effector outside workspace")
                    continue
                
                # Check emergency conditions (pass zero velocities since we're position-based)
                is_emergency, emergency_reason = self.safety.check_emergency_conditions(
                    q_safe, np.zeros(self.robot_model.n())
                )
                if is_emergency:
                    self.stats['safety_violations'] += 1
                    print(f"Safety violation: {emergency_reason}")
                    continue
                
                # Update current joint state
                self.robot_q = q_safe
                
                # Send to hardware
                joints_deg = np.rad2deg(q_safe).tolist()
                if not self.use_sim:
                    if self.driver.send_joint_targets(joints_deg):
                        self.stats['packets_sent'] += 1
                else:
                    # Simulation: update null driver
                    for i, angle in enumerate(q_safe):
                        self.driver.move_to(i, angle)
                    self.stats['packets_sent'] += 1
                
            except Exception as e:
                print(f"Control loop error: {e}")
                import traceback
                traceback.print_exc()
            
            # Timing
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"Warning: Control loop lagging by {abs(sleep_time)*1000:.1f}ms")
    
    def _pose_to_transform(self, pose) -> np.ndarray:
        """Convert target pose to 4x4 transformation matrix."""
        T = np.eye(4)
        T[:3, 3] = pose.position
        
        # For now, use identity rotation (just position control)
        # TODO: Add orientation control using pose.orientation_roll
        # This would require constructing a rotation matrix from roll angle
        # For 5-DOF arm, we might only control position and wrist roll
        
        return T
    
    def _clamp_workspace(self, T: np.ndarray) -> np.ndarray:
        """Clamp target pose to workspace bounds."""
        workspace = self.robot_config.get('workspace', {})
        if not workspace:
            return T
        
        pos = T[:3, 3]
        x_min = workspace.get('x_min', -np.inf)
        x_max = workspace.get('x_max', np.inf)
        y_min = workspace.get('y_min', -np.inf)
        y_max = workspace.get('y_max', np.inf)
        z_min = workspace.get('z_min', -np.inf)
        z_max = workspace.get('z_max', np.inf)
        
        clamped_pos = np.array([
            np.clip(pos[0], x_min, x_max),
            np.clip(pos[1], y_min, y_max),
            np.clip(pos[2], z_min, z_max)
        ])
        
        T[:3, 3] = clamped_pos
        return T
    
    def _execute_home(self):
        """Execute homing sequence."""
        print("Executing home command...")
        if not self.use_sim:
            self.driver.send_home_command()
            # Wait a bit for homing to complete
            time.sleep(2.0)
            # Reset pose to home
            T_home = self.kin.fk(np.array([js.home for js in self.robot_model.joints]))
            pos_home = T_home[:3, 3]
            self.joystick_mapper.reset_pose(pos_home, roll=0.0)
            self.robot_q = np.array([js.home for js in self.robot_model.joints])
            print("Home command sent")
    
    def _execute_park(self):
        """Execute park command."""
        print("Executing park command...")
        if not self.use_sim:
            self.driver.send_park_command()
            print("Park command sent")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current controller status."""
        uptime = 0
        if self.stats['start_time']:
            uptime = time.time() - self.stats['start_time']
        
        return {
            'running': self.is_running,
            'uptime_seconds': uptime,
            'ik_solves': self.stats['ik_solves'],
            'ik_failures': self.stats['ik_failures'],
            'safety_violations': self.stats['safety_violations'],
            'packets_sent': self.stats['packets_sent'],
            'robot_joint_angles_deg': np.rad2deg(self.robot_q).tolist(),
            'target_position': self.target_pose.position.tolist() if self.target_pose else None,
            'motion_scale': self.joystick_mapper.get_motion_scale()
        }
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


