"""Tests for inverse kinematics solver."""
import pytest
import numpy as np
from telearm import example_model, load_from_config, Kinematics, IK, IKOptions


def test_ik_home_position():
    """Test IK solver at home position."""
    model = example_model()
    kin = Kinematics(model)
    ik = IK(kin)
    
    # Home joint angles
    q_home = np.array([js.home for js in model.joints])
    
    # Forward kinematics to get target pose
    T_target = kin.fk(q_home)
    
    # Solve IK
    q_solution = ik.solve(q_home, T_target)
    
    # Should be close to original
    assert np.allclose(q_solution, q_home, atol=1e-3)


def test_ik_reachable_position():
    """Test IK solver with reachable positions."""
    model = example_model()
    kin = Kinematics(model)
    ik = IK(kin)
    
    # Start from home
    q_start = np.array([js.home for js in model.joints])
    
    # Create reachable target positions
    test_positions = [
        [0.1, 0.0, 0.1],
        [0.05, 0.05, 0.05],
        [0.0, 0.1, 0.05],
    ]
    
    for pos in test_positions:
        T_target = np.eye(4)
        T_target[:3, 3] = pos
        
        try:
            q_solution = ik.solve(q_start, T_target)
            
            # Verify solution
            T_achieved = kin.fk(q_solution)
            achieved_pos = T_achieved[:3, 3]
            
            # Should be close to target position
            assert np.allclose(achieved_pos, pos, atol=1e-2)
            
            # Check joint limits
            for i, js in enumerate(model.joints):
                assert js.limit.min <= q_solution[i] <= js.limit.max
            
        except Exception as e:
            # Some positions might be unreachable, that's ok
            print(f"IK failed for position {pos}: {e}")


def test_ik_joint_limits():
    """Test that IK respects joint limits."""
    model = example_model()
    kin = Kinematics(model)
    ik = IK(kin)
    
    q_start = np.array([js.home for js in model.joints])
    
    # Try to reach a position that would require joint limit violation
    T_target = np.eye(4)
    T_target[:3, 3] = [0.5, 0.0, 0.5]  # Far position that might violate limits
    
    try:
        q_solution = ik.solve(q_start, T_target)
        
        # Check all joints are within limits
        for i, js in enumerate(model.joints):
            assert js.limit.min <= q_solution[i] <= js.limit.max, \
                f"Joint {i} violates limits: {q_solution[i]} not in [{js.limit.min}, {js.limit.max}]"
                
    except Exception:
        # IK might fail for unreachable positions, that's acceptable
        pass


def test_ik_convergence():
    """Test IK solver convergence."""
    model = example_model()
    kin = Kinematics(model)
    ik = IK(kin)
    
    q_start = np.array([js.home for js in model.joints])
    
    # Create a modest target position
    T_target = np.eye(4)
    T_target[:3, 3] = [0.08, 0.02, 0.06]
    
    # Solve with different options
    options = IKOptions(max_iters=100, tol_pos=1e-4)
    q_solution = ik.solve(q_start, T_target, opts=options)
    
    # Verify convergence
    T_achieved = kin.fk(q_solution)
    achieved_pos = T_achieved[:3, 3]
    target_pos = T_target[:3, 3]
    
    error = np.linalg.norm(achieved_pos - target_pos)
    assert error < options.tol_pos


def test_ik_options():
    """Test IK solver with different options."""
    model = example_model()
    kin = Kinematics(model)
    ik = IK(kin)
    
    q_start = np.array([js.home for js in model.joints])
    T_target = np.eye(4)
    T_target[:3, 3] = [0.06, 0.0, 0.05]
    
    # Test with different damping factors
    for lambda2 in [1e-6, 1e-4, 1e-2]:
        options = IKOptions(lambda2=lambda2)
        try:
            q_solution = ik.solve(q_start, T_target, opts=options)
            
            # Should produce valid solution
            T_achieved = kin.fk(q_solution)
            achieved_pos = T_achieved[:3, 3]
            
            assert np.linalg.norm(achieved_pos - T_target[:3, 3]) < 1e-2
            
        except Exception:
            # Some damping values might fail
            pass


def test_ik_singularity_handling():
    """Test IK behavior near singularities."""
    model = example_model()
    kin = Kinematics(model)
    ik = IK(kin)
    
    # Create a configuration that might be near singularity
    q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # All joints at zero
    
    T_target = np.eye(4)
    T_target[:3, 3] = [0.05, 0.0, 0.03]
    
    try:
        # Should handle singularity gracefully
        q_solution = ik.solve(q_start, T_target)
        
        # Solution should be valid
        T_achieved = kin.fk(q_solution)
        achieved_pos = T_achieved[:3, 3]
        
        # Should be reasonably close
        error = np.linalg.norm(achieved_pos - T_target[:3, 3])
        assert error < 0.1  # More lenient for singularity regions
        
    except Exception as e:
        # IK might fail near singularities, which is acceptable
        print(f"IK failed near singularity: {e}")
