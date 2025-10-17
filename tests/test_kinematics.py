"""Tests for forward kinematics and Jacobian computation."""
import pytest
import numpy as np
from telearm import example_model, load_from_config, Kinematics


def test_fk_home():
    """Test forward kinematics at home position."""
    model = example_model()
    kin = Kinematics(model)
    q = np.array([js.home for js in model.joints])
    T = kin.fk(q)
    
    assert T.shape == (4, 4)
    assert np.allclose(T[3, :], [0, 0, 0, 1])  # Bottom row should be [0,0,0,1]
    assert np.allclose(np.linalg.det(T[:3, :3]), 1.0)  # Rotation matrix determinant


def test_fk_identity():
    """Test FK with zero joint angles."""
    model = example_model()
    kin = Kinematics(model)
    q = np.zeros(model.n())
    T = kin.fk(q)
    
    # Should not be identity due to DH parameters, but should be valid transform
    assert T.shape == (4, 4)
    assert np.allclose(T[3, :], [0, 0, 0, 1])


def test_jacobian_shape():
    """Test Jacobian matrix dimensions."""
    model = example_model()
    kin = Kinematics(model)
    q = np.array([js.home for js in model.joints])
    
    J = kin.jacobian_geometric(q)
    assert J.shape == (6, model.n())  # 6DOF x n joints


def test_jacobian_linearity():
    """Test Jacobian linearity around small perturbations."""
    model = example_model()
    kin = Kinematics(model)
    q0 = np.array([js.home for js in model.joints])
    
    # Small perturbation
    dq = 0.01 * np.random.randn(model.n())
    q1 = q0 + dq
    
    # Forward kinematics
    T0 = kin.fk(q0)
    T1 = kin.fk(q1)
    
    # Jacobian-based prediction
    J = kin.jacobian_geometric(q0)
    predicted_dx = J @ dq
    
    # Actual displacement
    actual_dx = np.concatenate([
        T1[:3, 3] - T0[:3, 3],  # Position change
        np.zeros(3)  # Simplified: ignore orientation change for this test
    ])
    
    # Should be approximately equal for small perturbations
    assert np.allclose(predicted_dx[:3], actual_dx[:3], atol=1e-3)


def test_config_loading():
    """Test loading model from config file."""
    try:
        model = load_from_config()
        assert model.n() == 5
        assert len(model.joints) == 5
        assert len(model.dh) == 5
        
        # Test that joints have proper structure
        for joint in model.joints:
            assert hasattr(joint, 'name')
            assert hasattr(joint, 'home')
            assert hasattr(joint, 'limit')
            assert joint.limit.min <= joint.limit.max
            
    except FileNotFoundError:
        pytest.skip("Config file not found, using example model")


def test_kinematics_consistency():
    """Test that FK is consistent with DH parameters."""
    model = example_model()
    kin = Kinematics(model)
    
    # Test at several random configurations
    for _ in range(10):
        q = np.random.uniform(-np.pi, np.pi, model.n())
        
        # Enforce joint limits
        for i, js in enumerate(model.joints):
            q[i] = np.clip(q[i], js.limit.min, js.limit.max)
        
        T = kin.fk(q)
        
        # Basic sanity checks
        assert T.shape == (4, 4)
        assert np.allclose(T[3, :], [0, 0, 0, 1])
        assert np.allclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-10)
