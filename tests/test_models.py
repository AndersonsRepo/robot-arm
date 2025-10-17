"""Tests for data models and configuration loading."""
import pytest
import numpy as np
import yaml
import tempfile
import os
from telearm import JointLimit, JointSpec, DH, ArmModel, example_model, load_robot_config, load_from_config


def test_joint_limit():
    """Test JointLimit dataclass."""
    limit = JointLimit(min=-np.pi, max=np.pi)
    
    assert limit.min == -np.pi
    assert limit.max == np.pi
    assert limit.min < limit.max


def test_joint_spec():
    """Test JointSpec dataclass."""
    limit = JointLimit(min=-np.pi/2, max=np.pi/2)
    spec = JointSpec("test_joint", home=0.0, limit=limit)
    
    assert spec.name == "test_joint"
    assert spec.home == 0.0
    assert spec.limit == limit
    assert spec.max_vel > 0
    assert spec.max_acc > 0


def test_dh_parameters():
    """Test DH parameter dataclass."""
    dh = DH(a=0.1, alpha=np.pi/2, d=0.05, theta_offset=0.0)
    
    assert dh.a == 0.1
    assert dh.alpha == np.pi/2
    assert dh.d == 0.05
    assert dh.theta_offset == 0.0


def test_arm_model():
    """Test ArmModel creation and basic properties."""
    joints = (
        JointSpec("joint1", home=0.0, limit=JointLimit(-np.pi, np.pi)),
        JointSpec("joint2", home=0.0, limit=JointLimit(-np.pi/2, np.pi/2)),
    )
    dh = (
        DH(a=0.1, alpha=np.pi/2, d=0.05),
        DH(a=0.2, alpha=0.0, d=0.0),
    )
    
    model = ArmModel(joints=joints, dh=dh)
    
    assert model.n() == 2
    assert len(model.joints) == 2
    assert len(model.dh) == 2


def test_example_model():
    """Test example model creation."""
    model = example_model()
    
    assert model.n() == 5
    assert len(model.joints) == 5
    assert len(model.dh) == 5
    
    # Check joint names
    expected_names = ["base_yaw", "base_pitch", "elbow", "wrist_pitch", "wrist_roll"]
    for i, joint in enumerate(model.joints):
        assert joint.name == expected_names[i]
    
    # Check joint limits are reasonable
    for joint in model.joints:
        assert joint.limit.min < joint.limit.max
        assert joint.limit.min >= -2*np.pi
        assert joint.limit.max <= 2*np.pi


def test_config_file_creation():
    """Test creating a config file and loading it."""
    config_data = {
        'robot': {
            'name': 'test-arm',
            'dof': 2
        },
        'joints': [
            {
                'name': 'joint1',
                'home': 0.0,
                'limit_min': -180.0,
                'limit_max': 180.0,
                'max_vel': 90.0,
                'max_acc': 180.0
            },
            {
                'name': 'joint2',
                'home': 0.0,
                'limit_min': -90.0,
                'limit_max': 90.0,
                'max_vel': 90.0,
                'max_acc': 180.0
            }
        ],
        'dh_parameters': {
            'convention': 'standard',
            'links': [
                {'a': 0.1, 'alpha': 90.0, 'd': 0.05, 'theta_offset': 0.0},
                {'a': 0.2, 'alpha': 0.0, 'd': 0.0, 'theta_offset': 0.0}
            ]
        }
    }
    
    # Create temporary config file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config_data, f)
        temp_config_path = f.name
    
    try:
        # Test loading config
        loaded_config = load_robot_config(temp_config_path)
        assert loaded_config['robot']['name'] == 'test-arm'
        assert loaded_config['robot']['dof'] == 2
        assert len(loaded_config['joints']) == 2
        assert len(loaded_config['dh_parameters']['links']) == 2
        
        # Test creating model from config
        model = load_from_config(temp_config_path)
        assert model.n() == 2
        assert model.joints[0].name == 'joint1'
        assert model.joints[1].name == 'joint2'
        
    finally:
        os.unlink(temp_config_path)


def test_config_loading_validation():
    """Test config loading with invalid data."""
    # Test missing required fields
    invalid_config = {
        'robot': {'name': 'test'},
        'joints': [{'name': 'joint1'}],  # Missing required fields
        'dh_parameters': {'links': []}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(invalid_config, f)
        temp_config_path = f.name
    
    try:
        with pytest.raises(KeyError):
            load_from_config(temp_config_path)
    finally:
        os.unlink(temp_config_path)


def test_model_immutability():
    """Test that model components are immutable where expected."""
    model = example_model()
    
    # JointSpec should be frozen
    with pytest.raises(AttributeError):
        model.joints[0].name = "new_name"
    
    # JointLimit should be frozen
    with pytest.raises(AttributeError):
        model.joints[0].limit.min = 0.0
    
    # DH should be frozen
    with pytest.raises(AttributeError):
        model.dh[0].a = 0.5


def test_degree_radian_conversion():
    """Test that config loading properly converts degrees to radians."""
    config_data = {
        'robot': {'name': 'test', 'dof': 1},
        'joints': [
            {
                'name': 'test_joint',
                'home': 90.0,  # degrees
                'limit_min': -180.0,  # degrees
                'limit_max': 180.0,   # degrees
                'max_vel': 180.0,     # deg/s
                'max_acc': 360.0      # deg/s^2
            }
        ],
        'dh_parameters': {
            'convention': 'standard',
            'links': [
                {'a': 0.1, 'alpha': 90.0, 'd': 0.05, 'theta_offset': 45.0}  # degrees
            ]
        }
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config_data, f)
        temp_config_path = f.name
    
    try:
        model = load_from_config(temp_config_path)
        
        # Check degree to radian conversion
        joint = model.joints[0]
        assert abs(joint.home - np.pi/2) < 1e-10  # 90 degrees
        assert abs(joint.limit.min - (-np.pi)) < 1e-10  # -180 degrees
        assert abs(joint.limit.max - np.pi) < 1e-10  # 180 degrees
        assert abs(joint.max_vel - np.pi) < 1e-10  # 180 deg/s
        assert abs(joint.max_acc - 2*np.pi) < 1e-10  # 360 deg/s^2
        
        dh = model.dh[0]
        assert abs(dh.alpha - np.pi/2) < 1e-10  # 90 degrees
        assert abs(dh.theta_offset - np.pi/4) < 1e-10  # 45 degrees
        
    finally:
        os.unlink(temp_config_path)
