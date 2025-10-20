"""
Main Teleoperation Controller

Real-time teleoperation controller that coordinates all components:
- Network receiver for operator data
- Velocity mapping from operator to robot
- Safety checking and constraints
- Hardware control via servo drivers
"""
from __future__ import annotations
import time
import threading
import yaml
from typing import Dict, Any, Optional
import numpy as np

from ..models import load_from_config, load_operator_from_config
from ..network.receiver import OperatorDataReceiver, MockOperatorDataReceiver, create_receiver
from ..safety.checker import SafetyChecker, EmergencyStopHandler
from ..teleoperation.mapper import VelocityMapper
from ..teleoperation.integrator import VelocityIntegrator
from ..drivers import SerialServoDriver, NullServoDriver


class TeleopController:
    """Main teleoperation controller."""
    
    def __init__(self, config_path: str = "config/teleop.yaml", use_mock: bool = False, config_override: Optional[Dict[str, Any]] = None):
        """
        Initialize teleoperation controller.
        
        Args:
            config_path: Path to teleoperation configuration file
            use_mock: Use mock receiver for testing without hardware
            config_override: Optional config dictionary to override file settings
        """
        # Load configurations
        self.config = self._load_config(config_path)
        
        # Apply config overrides if provided
        if config_override:
            self.config.update(config_override)
        self.robot_model = load_from_config()
        self.operator_model = load_operator_from_config()
        
        # Initialize components
        self.mapper = VelocityMapper(self.operator_model, self.robot_model, self.config)
        self.safety = SafetyChecker(self.robot_model, self.config)
        self.emergency_handler = EmergencyStopHandler(self.config)
        self.integrator = VelocityIntegrator(self.robot_model, dt=1.0/self.config['update_rate_hz'])
        
        # Network receiver - use factory for mode selection
        self.receiver = create_receiver(self.config, use_mock=use_mock)
        
        # Hardware driver
        self.driver = self._create_driver()
        
        # State
        self.robot_q = np.array([js.home for js in self.robot_model.joints])
        self.robot_dq = np.zeros(self.robot_model.n())
        self.is_running = False
        self.control_thread = None
        
        # Statistics
        self.stats = {
            'packets_received': 0,
            'packets_processed': 0,
            'safety_violations': 0,
            'start_time': None,
            'last_packet_time': None
        }
        
        # Set up callbacks
        self.receiver.set_packet_callback(self._on_packet_received)
        self.receiver.set_timeout_callback(self._on_connection_timeout)
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load teleoperation configuration."""
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load config from {config_path}: {e}")
            # Return default configuration
            return {
                'velocity_scale': 0.6,
                'update_rate_hz': 100,
                'timeout_ms': 200,
                'mapping': {'null_space_gain': 0.1},
                'safety': {
                    'enforce_joint_limits': True,
                    'enforce_workspace': True,
                    'soft_limit_margin_deg': 5.0,
                    'emergency_stop_pin': 13
                }
            }
    
    def _create_driver(self):
        """Create appropriate servo driver."""
        try:
            # Try to create serial driver
            return SerialServoDriver(
                port="/dev/ttyUSB0",  # Default port
                baud_rate=115200,
                timeout=0.25
            )
        except Exception as e:
            print(f"Warning: Could not create serial driver: {e}")
            print("Using null driver for simulation")
            return NullServoDriver()
    
    def start(self):
        """Start the teleoperation controller."""
        if self.is_running:
            return
        
        print("Starting teleoperation controller...")
        
        # Initialize integrator to current robot position
        self.integrator.set_positions(self.robot_q)
        
        # Start network receiver
        self.receiver.start()
        
        # Start control loop
        self.is_running = True
        self.stats['start_time'] = time.time()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print("Teleoperation controller started")
    
    def stop(self):
        """Stop the teleoperation controller."""
        if not self.is_running:
            return
        
        print("Stopping teleoperation controller...")
        
        self.is_running = False
        
        # Stop receiver
        self.receiver.stop()
        
        # Wait for control thread to finish
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        # Move to home position
        self._go_home()
        
        print("Teleoperation controller stopped")
    
    def _control_loop(self):
        """Main control loop running at specified rate."""
        rate = self.config['update_rate_hz']
        dt = 1.0 / rate
        
        print(f"Control loop started at {rate} Hz")
        
        while self.is_running:
            loop_start = time.time()
            
            try:
                # Get latest operator data
                packet = self.receiver.get_latest()
                if packet is None:
                    self._handle_no_data()
                    time.sleep(dt)
                    continue
                
                # Update statistics
                self.stats['packets_received'] += 1
                self.stats['last_packet_time'] = time.time()
                
                # Check emergency conditions
                if self.emergency_handler.is_emergency_active():
                    self._handle_emergency()
                    time.sleep(dt)
                    continue
                
                # Update emergency handler watchdog
                self.emergency_handler.update_command_time()
                
                # Map operator velocities to robot velocities
                robot_dq_desired = self.mapper.map_velocity(
                    packet.operator_joints,
                    packet.operator_velocities,
                    self.robot_q
                )
                
                # Apply safety checks
                is_safe, robot_dq_safe, status_msg = self.safety.comprehensive_safety_check(
                    self.robot_q, robot_dq_desired, self.robot_dq, dt
                )
                
                if not is_safe:
                    self.stats['safety_violations'] += 1
                    print(f"Safety violation: {status_msg}")
                    self._handle_safety_violation()
                    time.sleep(dt)
                    continue
                
                # Integrate velocities to positions
                self.robot_q, self.robot_dq = self.integrator.integrate(robot_dq_safe)
                
                # Send commands to hardware
                self._send_to_hardware()
                
                # Update statistics
                self.stats['packets_processed'] += 1
                
                # Debug output
                if self.stats['packets_processed'] % 100 == 0:
                    print(f"Processed {self.stats['packets_processed']} packets, "
                          f"status: {status_msg}, "
                          f"joints: {np.rad2deg(self.robot_q)}")
                
            except Exception as e:
                print(f"Control loop error: {e}")
                self._handle_control_error()
            
            # Timing
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Control loop is running slower than desired
                print(f"Warning: Control loop lagging by {abs(sleep_time)*1000:.1f}ms")
    
    def _on_packet_received(self, packet):
        """Callback for received packets."""
        pass  # Handled in control loop
    
    def _on_connection_timeout(self):
        """Callback for connection timeout."""
        print("Connection timeout - no operator data received")
        self._handle_no_data()
    
    def _handle_no_data(self):
        """Handle case when no operator data is available."""
        # Stop robot motion
        self.robot_dq = np.zeros(self.robot_model.n())
        self.integrator.set_positions(self.robot_q)
    
    def _handle_emergency(self):
        """Handle emergency stop condition."""
        print("EMERGENCY STOP ACTIVE")
        self.robot_dq = np.zeros(self.robot_model.n())
        self.integrator.set_positions(self.robot_q)
        # Hardware will be disabled by Arduino watchdog
    
    def _handle_safety_violation(self):
        """Handle safety violation."""
        self.robot_dq = np.zeros(self.robot_model.n())
        self.integrator.set_positions(self.robot_q)
    
    def _handle_control_error(self):
        """Handle control loop error."""
        self.robot_dq = np.zeros(self.robot_model.n())
        self.integrator.set_positions(self.robot_q)
    
    def _send_to_hardware(self):
        """Send joint positions to hardware."""
        try:
            for i, angle in enumerate(self.robot_q):
                self.driver.move_to(i, angle)
        except Exception as e:
            print(f"Hardware communication error: {e}")
    
    def _go_home(self):
        """Move robot to home position."""
        print("Moving to home position...")
        home_q = np.array([js.home for js in self.robot_model.joints])
        
        # Generate trajectory to home
        from ..trajectory import Trajectory
        traj = Trajectory()
        q_traj, t_traj = traj.cubic_time_scaling(
            self.robot_q, home_q, duration=2.0, dt=0.01
        )
        
        # Execute trajectory
        for q, t in zip(q_traj, t_traj):
            for i, angle in enumerate(q):
                self.driver.move_to(i, angle)
            time.sleep(0.01)
        
        self.robot_q = home_q
        self.robot_dq = np.zeros(self.robot_model.n())
        print("Home position reached")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current controller status."""
        uptime = 0
        if self.stats['start_time']:
            uptime = time.time() - self.stats['start_time']
        
        return {
            'running': self.is_running,
            'uptime_seconds': uptime,
            'packets_received': self.stats['packets_received'],
            'packets_processed': self.stats['packets_processed'],
            'safety_violations': self.stats['safety_violations'],
            'connection_alive': self.receiver.is_connection_alive(),
            'emergency_active': self.emergency_handler.is_emergency_active(),
            'robot_position': np.rad2deg(self.robot_q).tolist(),
            'robot_velocity': np.rad2deg(self.robot_dq).tolist(),
            'network_stats': self.receiver.get_stats().__dict__
        }
    
    def reset_emergency(self):
        """Reset emergency stop condition."""
        self.emergency_handler.reset_emergency()
        self.safety.reset_emergency()
        print("Emergency stop reset")


def test_teleop_controller():
    """Test the teleoperation controller with mock data."""
    print("Testing TeleopController...")
    
    # Create controller with mock receiver
    controller = TeleopController(use_mock=True)
    
    try:
        # Start controller
        controller.start()
        
        # Let it run for a few seconds
        time.sleep(3.0)
        
        # Get status
        status = controller.get_status()
        print(f"Controller status: {status}")
        
        # Stop controller
        controller.stop()
        
    except Exception as e:
        print(f"Test error: {e}")
        controller.stop()
    
    print("TeleopController test completed")


if __name__ == "__main__":
    test_teleop_controller()
