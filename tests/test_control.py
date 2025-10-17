"""Tests for motion controller."""
import pytest
import numpy as np
from telearm import example_model, NullServoDriver, MotionController


class MockServoDriver:
    """Mock servo driver for testing."""
    
    def __init__(self, n_joints):
        self.n_joints = n_joints
        self._angles = np.zeros(n_joints)
        self._move_calls = []
    
    def move_to(self, idx, angle_rad, speed=None):
        self._move_calls.append((idx, angle_rad, speed))
        self._angles[idx] = angle_rad
    
    def angles(self):
        return self._angles.copy()


def test_motion_controller_creation():
    """Test motion controller initialization."""
    model = example_model()
    driver = NullServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    assert ctrl.model == model
    assert ctrl.driver == driver
    assert ctrl.model.n() == 5


def test_go_home():
    """Test moving to home position."""
    model = example_model()
    driver = MockServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    ctrl.go_home()
    
    # Check that all joints were moved to home positions
    assert len(driver._move_calls) == model.n()
    
    for i, joint in enumerate(model.joints):
        # Find the call for this joint
        joint_calls = [call for call in driver._move_calls if call[0] == i]
        assert len(joint_calls) == 1
        assert joint_calls[0][1] == joint.home


def test_move_cartesian_basic():
    """Test basic Cartesian motion."""
    model = example_model()
    driver = MockServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    # Start at home
    q_start = np.array([js.home for js in model.joints])
    
    # Target position
    T_goal = np.eye(4)
    T_goal[:3, 3] = [0.05, 0.0, 0.04]  # Small movement
    
    # Execute motion
    q_final = ctrl.move_cartesian(q_start, T_goal, seconds=1.0, steps=10)
    
    # Should have made multiple move calls
    assert len(driver._move_calls) > model.n()  # More than just initial moves
    
    # Final angles should be valid
    assert len(q_final) == model.n()
    for i, angle in enumerate(q_final):
        assert model.joints[i].limit.min <= angle <= model.joints[i].limit.max


def test_move_cartesian_joint_limits():
    """Test that Cartesian motion respects joint limits."""
    model = example_model()
    driver = MockServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    q_start = np.array([js.home for js in model.joints])
    
    # Try to move to a position that might violate limits
    T_goal = np.eye(4)
    T_goal[:3, 3] = [0.3, 0.3, 0.3]  # Large movement
    
    q_final = ctrl.move_cartesian(q_start, T_goal, seconds=0.5, steps=5)
    
    # All final angles should be within limits
    for i, angle in enumerate(q_final):
        assert model.joints[i].limit.min <= angle <= model.joints[i].limit.max


def test_move_cartesian_trajectory():
    """Test that Cartesian motion follows a trajectory."""
    model = example_model()
    driver = MockServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    q_start = np.array([js.home for js in model.joints])
    
    T_goal = np.eye(4)
    T_goal[:3, 3] = [0.08, 0.02, 0.06]
    
    # Execute motion with few steps to make it observable
    q_final = ctrl.move_cartesian(q_start, T_goal, seconds=1.0, steps=3)
    
    # Should have made moves for each step
    expected_calls = 3 * model.n()  # 3 steps * 5 joints
    assert len(driver._move_calls) >= expected_calls
    
    # Verify that angles change over time (trajectory)
    joint_angles_over_time = {}
    for idx, angle, speed in driver._move_calls:
        if idx not in joint_angles_over_time:
            joint_angles_over_time[idx] = []
        joint_angles_over_time[idx].append(angle)
    
    # Each joint should have multiple different angles
    for joint_idx, angles in joint_angles_over_time.items():
        assert len(set(angles)) > 1  # Should have some variation


def test_motion_controller_components():
    """Test that motion controller has all required components."""
    model = example_model()
    driver = NullServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    # Should have kinematics
    assert hasattr(ctrl, 'kin')
    assert ctrl.kin is not None
    
    # Should have IK solver
    assert hasattr(ctrl, 'ik')
    assert ctrl.ik is not None
    
    # Should have trajectory generator
    assert hasattr(ctrl, 'trajectory')
    assert ctrl.trajectory is not None


def test_move_cartesian_validation():
    """Test Cartesian motion input validation."""
    model = example_model()
    driver = MockServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    q_start = np.array([js.home for js in model.joints])
    
    # Test with invalid transform matrix
    T_invalid = np.eye(3)  # Wrong shape
    
    with pytest.raises((IndexError, ValueError)):
        ctrl.move_cartesian(q_start, T_invalid, seconds=1.0)
    
    # Test with invalid start angles
    q_invalid = np.array([0.0, 0.0])  # Wrong length
    
    T_goal = np.eye(4)
    T_goal[:3, 3] = [0.05, 0.0, 0.04]
    
    with pytest.raises((IndexError, ValueError)):
        ctrl.move_cartesian(q_invalid, T_goal, seconds=1.0)


def test_move_cartesian_parameters():
    """Test Cartesian motion with different parameters."""
    model = example_model()
    driver = MockServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    q_start = np.array([js.home for js in model.joints])
    T_goal = np.eye(4)
    T_goal[:3, 3] = [0.06, 0.0, 0.05]
    
    # Test different time durations
    for seconds in [0.5, 1.0, 2.0]:
        driver._move_calls.clear()
        q_final = ctrl.move_cartesian(q_start, T_goal, seconds=seconds, steps=5)
        
        # Should complete successfully
        assert len(q_final) == model.n()
        
        # Should have made moves
        assert len(driver._move_calls) > 0
    
    # Test different step counts
    for steps in [5, 10, 20]:
        driver._move_calls.clear()
        q_final = ctrl.move_cartesian(q_start, T_goal, seconds=1.0, steps=steps)
        
        # More steps should result in more move calls
        expected_min_calls = steps * model.n()
        assert len(driver._move_calls) >= expected_min_calls


def test_null_driver_integration():
    """Test motion controller with NullServoDriver."""
    model = example_model()
    driver = NullServoDriver(model.n())
    ctrl = MotionController(model, driver)
    
    # Test home motion
    ctrl.go_home()
    angles = driver.angles()
    assert len(angles) == model.n()
    
    # Test Cartesian motion
    q_start = driver.angles()
    T_goal = np.eye(4)
    T_goal[:3, 3] = [0.05, 0.0, 0.04]
    
    q_final = ctrl.move_cartesian(q_start, T_goal, seconds=1.0, steps=10)
    
    # Should complete without error
    assert len(q_final) == model.n()
    assert np.all(np.isfinite(q_final))
