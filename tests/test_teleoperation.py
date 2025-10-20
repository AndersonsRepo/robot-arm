"""
Tests for teleoperation system components.

Tests network protocol, velocity mapping, safety checking,
IMU fusion algorithms, and integration with mock data.
Note: IMU fusion runs on ESP32 firmware in production, but
Python implementation is available for testing and development.
"""
import pytest
import numpy as np
import time
from unittest.mock import Mock, patch

from telearm.models import load_from_config, load_operator_from_config
from telearm.sensors import IMUReading, OperatorPose, TeleopPacket, create_mock_imu_reading, create_mock_operator_pose
from telearm.network.protocol import TeleopProtocol, NetworkStats
from telearm.network.receiver import MockOperatorDataReceiver
from telearm.teleoperation.mapper import VelocityMapper, CartesianVelocityMapper
from telearm.teleoperation.integrator import VelocityIntegrator, SmoothVelocityIntegrator
from telearm.safety.checker import SafetyChecker, EmergencyStopHandler
from telearm.teleoperation.controller import TeleopController
from telearm.imu_fusion import ComplementaryFilter, OperatorPoseEstimator, create_mock_imu_fusion_data


class TestNetworkProtocol:
    """Test network communication protocol."""
    
    def test_packet_packing_unpacking(self):
        """Test packet serialization/deserialization."""
        protocol = TeleopProtocol()
        
        # Create test packet
        packet = TeleopPacket(
            sequence=123,
            timestamp=time.time(),
            operator_joints=np.array([0.1, 0.2, 0.3]),
            operator_velocities=np.array([0.05, 0.1, 0.15]),
            confidence=0.9
        )
        
        # Pack and unpack
        packed = protocol.pack_packet(packet)
        unpacked = protocol.unpack_packet(packed)
        
        assert unpacked is not None
        assert packet.sequence == unpacked.sequence
        assert np.allclose(packet.operator_joints, unpacked.operator_joints)
        assert np.allclose(packet.operator_velocities, unpacked.operator_velocities)
        assert abs(packet.confidence - unpacked.confidence) < 1e-6
    
    def test_packet_size(self):
        """Test that packets are exactly 40 bytes."""
        packet = TeleopPacket(
            sequence=0,
            timestamp=0.0,
            operator_joints=np.zeros(3),
            operator_velocities=np.zeros(3),
            confidence=0.0
        )
        
        packed = packet.pack()
        assert len(packed) == 40
    
    def test_network_stats(self):
        """Test network statistics tracking."""
        stats = NetworkStats()
        
        # Update latency
        stats.update_latency(0.0, 0.01)
        assert stats.latency_ms == 10.0
        
        # Update jitter
        stats.update_jitter(12.0)
        assert stats.jitter_ms == 2.0


class TestVelocityMapping:
    """Test velocity mapping from operator to robot."""
    
    def test_velocity_mapper(self):
        """Test velocity mapper with mock data."""
        robot_model = load_from_config()
        operator_model = load_operator_from_config()
        
        config = {
            'velocity_scale': 0.6,
            'mapping': {'null_space_gain': 0.1}
        }
        
        mapper = VelocityMapper(operator_model, robot_model, config)
        
        # Test data
        op_q = np.array([0.1, 0.2, 0.3])
        op_dq = np.array([0.05, 0.1, 0.15])
        robot_q = np.zeros(5)
        
        # Map velocity
        robot_dq = mapper.map_velocity(op_q, op_dq, robot_q)
        
        assert len(robot_dq) == 5
        assert np.all(np.isfinite(robot_dq))
    
    def test_cartesian_velocity_mapper(self):
        """Test Cartesian velocity mapping."""
        robot_model = load_from_config()
        config = {'velocity_scale': 0.6}
        
        mapper = CartesianVelocityMapper(robot_model, config)
        
        # Test Cartesian velocity mapping
        v_desired = np.array([0.1, 0.05, 0.02, 0.0, 0.0, 0.0])  # 6D twist
        robot_q = np.zeros(5)
        
        robot_dq = mapper.map_cartesian_velocity(v_desired, robot_q)
        
        assert len(robot_dq) == 5
        assert np.all(np.isfinite(robot_dq))


class TestVelocityIntegration:
    """Test velocity integration and constraints."""
    
    def test_velocity_integrator(self):
        """Test velocity integrator."""
        robot_model = load_from_config()
        integrator = VelocityIntegrator(robot_model, dt=0.01)
        
        # Test integration
        desired_velocities = np.array([0.1, 0.2, 0.3, 0.1, 0.05])
        
        positions, velocities = integrator.integrate(desired_velocities)
        
        assert len(positions) == 5
        assert len(velocities) == 5
        assert np.all(np.isfinite(positions))
        assert np.all(np.isfinite(velocities))
    
    def test_velocity_limits(self):
        """Test velocity limiting."""
        robot_model = load_from_config()
        integrator = VelocityIntegrator(robot_model, dt=0.01)
        
        # Test with extreme velocities
        extreme_velocities = np.array([10.0, 10.0, 10.0, 10.0, 10.0])
        
        positions, velocities = integrator.integrate(extreme_velocities)
        
        # Velocities should be limited
        for i, (vel, joint_spec) in enumerate(zip(velocities, robot_model.joints)):
            assert abs(vel) <= joint_spec.max_vel
    
    def test_smooth_integrator(self):
        """Test smooth velocity integrator."""
        robot_model = load_from_config()
        integrator = SmoothVelocityIntegrator(robot_model, dt=0.01, smoothing_factor=0.7)
        
        # Test integration
        desired_velocities = np.array([0.1, 0.2, 0.3, 0.1, 0.05])
        
        positions, velocities = integrator.integrate(desired_velocities)
        
        assert len(positions) == 5
        assert len(velocities) == 5


class TestSafetyChecking:
    """Test safety checking and constraints."""
    
    def test_safety_checker(self):
        """Test safety checker."""
        robot_model = load_from_config()
        config = {
            'safety': {
                'enforce_joint_limits': True,
                'enforce_workspace': True,
                'soft_limit_margin_deg': 5.0
            }
        }
        
        checker = SafetyChecker(robot_model, config)
        
        # Test with normal values
        q = np.zeros(5)
        dq = np.array([0.1, 0.2, 0.3, 0.1, 0.05])
        
        is_safe, corrected_dq = checker.check_joint_limits(q, dq)
        
        assert is_safe
        assert len(corrected_dq) == 5
    
    def test_velocity_limits(self):
        """Test velocity limiting."""
        robot_model = load_from_config()
        config = {'safety': {}}
        
        checker = SafetyChecker(robot_model, config)
        
        # Test with extreme velocities
        extreme_dq = np.array([10.0, 10.0, 10.0, 10.0, 10.0])
        limited_dq = checker.enforce_velocity_limits(extreme_dq)
        
        for i, (vel, joint_spec) in enumerate(zip(limited_dq, robot_model.joints)):
            assert abs(vel) <= joint_spec.max_vel
    
    def test_workspace_checking(self):
        """Test workspace boundary checking."""
        robot_model = load_from_config()
        config = {
            'safety': {
                'enforce_workspace': True
            }
        }
        
        checker = SafetyChecker(robot_model, config)
        
        # Test with identity matrix (should be within workspace)
        T_ee = np.eye(4)
        in_workspace = checker.check_workspace(T_ee)
        
        assert isinstance(in_workspace, bool)
    
    def test_emergency_handler(self):
        """Test emergency stop handler."""
        config = {
            'safety': {'emergency_stop_pin': 13},
            'timeout_ms': 200
        }
        
        handler = EmergencyStopHandler(config)
        
        # Test initial state
        assert not handler.is_emergency_active()
        
        # Test watchdog
        handler.update_command_time()
        assert handler.check_watchdog()


class TestMockReceiver:
    """Test mock operator data receiver."""
    
    def test_mock_receiver(self):
        """Test mock receiver functionality."""
        receiver = MockOperatorDataReceiver(port=5000, buffer_size=10)
        
        # Start receiver
        receiver.start()
        
        # Let it generate some packets
        time.sleep(0.1)
        
        # Check for packets
        packet = receiver.get_latest()
        assert packet is not None
        assert len(packet.operator_joints) == 3
        assert len(packet.operator_velocities) == 3
        
        # Check connection status
        assert receiver.is_connection_alive()
        
        # Stop receiver
        receiver.stop()
    
    def test_mock_receiver_callback(self):
        """Test mock receiver with callback."""
        received_packets = []
        
        def packet_callback(packet):
            received_packets.append(packet)
        
        receiver = MockOperatorDataReceiver(port=5000, buffer_size=10)
        receiver.set_packet_callback(packet_callback)
        
        # Start receiver
        receiver.start()
        
        # Let it generate some packets
        time.sleep(0.1)
        
        # Check that callbacks were called
        assert len(received_packets) > 0
        
        # Stop receiver
        receiver.stop()


class TestTeleopController:
    """Test teleoperation controller integration."""
    
    def test_controller_initialization(self):
        """Test controller initialization."""
        controller = TeleopController(use_mock=True)
        
        assert controller.robot_model is not None
        assert controller.operator_model is not None
        assert controller.mapper is not None
        assert controller.safety is not None
        assert controller.receiver is not None
    
    def test_controller_mock_mode(self):
        """Test controller with mock data."""
        controller = TeleopController(use_mock=True)
        
        # Start controller
        controller.start()
        
        # Let it run briefly
        time.sleep(0.5)
        
        # Get status
        status = controller.get_status()
        assert status['running']
        assert 'packets_received' in status
        assert 'packets_processed' in status
        
        # Stop controller
        controller.stop()
        
        # Verify stopped
        status = controller.get_status()
        assert not status['running']
    
    def test_controller_error_handling(self):
        """Test controller error handling."""
        controller = TeleopController(use_mock=True)
        
        # Start controller
        controller.start()
        
        # Let it run briefly
        time.sleep(0.1)
        
        # Stop controller
        controller.stop()
        
        # Should not raise exceptions


class TestIntegration:
    """Integration tests for teleoperation system."""
    
    def test_end_to_end_mock(self):
        """Test end-to-end system with mock data."""
        # This test simulates the complete teleoperation pipeline
        
        # 1. Create operator pose
        pose = create_mock_operator_pose()
        
        # 2. Create packet
        packet = TeleopPacket(
            sequence=1,
            timestamp=time.time(),
            operator_joints=pose.joint_angles,
            operator_velocities=pose.joint_velocities,
            confidence=pose.confidence
        )
        
        # 3. Test velocity mapping
        robot_model = load_from_config()
        operator_model = load_operator_from_config()
        config = {'velocity_scale': 0.6, 'mapping': {'null_space_gain': 0.1}}
        
        mapper = VelocityMapper(operator_model, robot_model, config)
        robot_q = np.zeros(5)
        robot_dq = mapper.map_velocity(packet.operator_joints, packet.operator_velocities, robot_q)
        
        # 4. Test safety checking
        safety_config = {'safety': {'enforce_joint_limits': True, 'enforce_workspace': True}}
        checker = SafetyChecker(robot_model, safety_config)
        is_safe, safe_dq, status = checker.comprehensive_safety_check(robot_q, robot_dq, np.zeros(5), 0.01)
        
        # 5. Test velocity integration
        integrator = VelocityIntegrator(robot_model, dt=0.01)
        new_positions, actual_velocities = integrator.integrate(safe_dq)
        
        # Verify results
        assert len(robot_dq) == 5
        assert len(safe_dq) == 5
        assert len(new_positions) == 5
        assert len(actual_velocities) == 5
        assert np.all(np.isfinite(new_positions))
        assert np.all(np.isfinite(actual_velocities))


class TestIMUFusion:
    """Test IMU fusion algorithms for mock data generation."""
    
    def test_complementary_filter_initialization(self):
        """Test complementary filter initialization."""
        filter_obj = ComplementaryFilter()
        assert filter_obj.alpha == 0.98
        assert filter_obj.gyro_threshold == 0.1
        assert not filter_obj.state.initialized
    
    def test_complementary_filter_update(self):
        """Test complementary filter update with gravity."""
        filter_obj = ComplementaryFilter()
        
        # Test with gravity vector
        accel = np.array([0, 0, 9.81])
        gyro = np.array([0, 0, 0])
        dt = 0.01
        
        quat = filter_obj.update(accel, gyro, dt)
        
        # Should be initialized after first update
        assert filter_obj.state.initialized
        assert quat is not None
        assert len(quat) == 4
        assert np.isclose(np.linalg.norm(quat), 1.0)  # Should be normalized
    
    def test_complementary_filter_rotation(self):
        """Test complementary filter with rotation."""
        filter_obj = ComplementaryFilter()
        
        # Test with rotation
        accel = np.array([0, 0, 9.81])
        gyro = np.array([0, 0, 1.0])  # 1 rad/s rotation
        dt = 0.01
        
        # Update multiple times
        for _ in range(10):
            quat = filter_obj.update(accel, gyro, dt)
        
        # Should have rotated
        assert quat is not None
        assert np.isclose(np.linalg.norm(quat), 1.0)
    
    def test_quaternion_to_euler(self):
        """Test quaternion to Euler angle conversion."""
        filter_obj = ComplementaryFilter()
        
        # Test identity quaternion
        q_identity = np.array([1, 0, 0, 0])
        euler = filter_obj.quaternion_to_euler(q_identity)
        
        assert len(euler) == 3
        assert np.allclose(euler, [0, 0, 0], atol=1e-6)
    
    def test_operator_pose_estimator_initialization(self):
        """Test operator pose estimator initialization."""
        estimator = OperatorPoseEstimator()
        assert estimator.num_imus == 3
        assert len(estimator.filters) == 3
        assert len(estimator.imu_orientations) == 3
    
    def test_operator_pose_estimator_update(self):
        """Test operator pose estimation from IMU readings."""
        estimator = OperatorPoseEstimator()
        
        # Create mock IMU readings
        imu_readings = []
        for i in range(3):
            reading = IMUReading(
                imu_id=i,
                timestamp=time.time(),
                accel=type('Accel', (), {'x': 0, 'y': 0, 'z': 9.81})(),
                gyro=type('Gyro', (), {'x': 0, 'y': 0, 'z': 0})(),
                mag=type('Mag', (), {'x': 0, 'y': 0, 'z': 0})(),
                temperature=25.0
            )
            imu_readings.append(reading)
        
        pose = estimator.update(imu_readings)
        
        assert pose is not None
        assert len(pose.joint_angles) == 3
        assert len(pose.joint_velocities) == 3
        assert pose.confidence > 0
    
    def test_mock_imu_fusion_data_generation(self):
        """Test mock IMU fusion data generation."""
        poses = create_mock_imu_fusion_data(num_samples=10)
        
        assert len(poses) == 10
        for pose in poses:
            assert pose is not None
            assert len(pose.joint_angles) == 3
            assert len(pose.joint_velocities) == 3
            assert pose.confidence > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
