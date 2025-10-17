"""Tests for trajectory generation."""
import pytest
import numpy as np
from telearm import Trajectory


def test_cubic_time_scaling():
    """Test cubic time scaling trajectory."""
    traj = Trajectory()
    
    # Simple linear motion
    p0 = np.array([0.0, 0.0, 0.0])
    pf = np.array([1.0, 0.0, 0.0])
    T = 2.0
    
    trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)
    
    # Test at start
    pos_start = trajectory_func(0.0)
    assert np.allclose(pos_start, p0)
    
    # Test at end
    pos_end = trajectory_func(T)
    assert np.allclose(pos_end, pf)
    
    # Test at midpoint
    pos_mid = trajectory_func(T/2)
    expected_mid = (p0 + pf) / 2
    assert np.allclose(pos_mid, expected_mid, atol=1e-10)


def test_trajectory_continuity():
    """Test that trajectory is continuous."""
    traj = Trajectory()
    
    p0 = np.array([0.0, 1.0, 2.0])
    pf = np.array([3.0, 4.0, 5.0])
    T = 1.5
    
    trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)
    
    # Test continuity at multiple points
    times = np.linspace(0, T, 100)
    positions = [trajectory_func(t) for t in times]
    
    # All positions should be finite
    for pos in positions:
        assert np.all(np.isfinite(pos))
    
    # Positions should be monotonically changing (for this simple case)
    x_positions = [pos[0] for pos in positions]
    assert np.all(np.diff(x_positions) >= 0)  # Monotonic increase


def test_trajectory_bounds():
    """Test that trajectory stays within bounds."""
    traj = Trajectory()
    
    p0 = np.array([-1.0, -2.0, -3.0])
    pf = np.array([1.0, 2.0, 3.0])
    T = 1.0
    
    trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)
    
    # Test at many points
    times = np.linspace(0, T, 50)
    for t in times:
        pos = trajectory_func(t)
        
        # Should be within the bounding box
        for i in range(3):
            assert p0[i] <= pos[i] <= pf[i] or pf[i] <= pos[i] <= p0[i]


def test_trajectory_derivatives():
    """Test trajectory derivatives (velocity approximation)."""
    traj = Trajectory()
    
    p0 = np.array([0.0, 0.0, 0.0])
    pf = np.array([1.0, 0.0, 0.0])
    T = 2.0
    
    trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)
    
    # Approximate velocity using finite differences
    dt = 0.001
    t = 1.0
    pos_t = trajectory_func(t)
    pos_t_dt = trajectory_func(t + dt)
    velocity = (pos_t_dt - pos_t) / dt
    
    # Velocity should be reasonable (not too large)
    assert np.linalg.norm(velocity) < 10.0  # Arbitrary reasonable limit


def test_trajectory_zero_time():
    """Test trajectory with zero time duration."""
    traj = Trajectory()
    
    p0 = np.array([1.0, 2.0, 3.0])
    pf = np.array([4.0, 5.0, 6.0])
    T = 0.0  # Zero time
    
    # Should raise ValueError for zero time
    with pytest.raises(ValueError, match="tf must be greater than t0"):
        trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)


def test_trajectory_same_start_end():
    """Test trajectory with same start and end points."""
    traj = Trajectory()
    
    p0 = np.array([1.0, 2.0, 3.0])
    pf = p0.copy()  # Same point
    T = 1.0
    
    trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)
    
    # Should stay at the same point
    times = np.linspace(0, T, 10)
    for t in times:
        pos = trajectory_func(t)
        assert np.allclose(pos, p0)


def test_trajectory_3d_motion():
    """Test 3D trajectory motion."""
    traj = Trajectory()
    
    p0 = np.array([0.0, 0.0, 0.0])
    pf = np.array([1.0, 1.0, 1.0])
    T = 2.0
    
    trajectory_func = traj.cubic_time_scaling(0.0, T, p0, pf)
    
    # Test motion in all three dimensions
    t = 1.0
    pos = trajectory_func(t)
    
    # Should be somewhere in the middle
    assert 0.0 < pos[0] < 1.0
    assert 0.0 < pos[1] < 1.0
    assert 0.0 < pos[2] < 1.0
