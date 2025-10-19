"""
Velocity-Based Mapping for Telemanipulation

Maps operator arm velocities to robot arm velocities using Jacobian-based control.
"""
from __future__ import annotations
from typing import Dict, Any
import numpy as np
from ..models import ArmModel, OperatorArmModel
from ..kinematics import Kinematics


class VelocityMapper:
    """Maps operator arm velocities to robot arm velocities."""
    
    def __init__(self, operator_model: OperatorArmModel, robot_model: ArmModel, config: Dict[str, Any]):
        """
        Initialize velocity mapper.
        
        Args:
            operator_model: Operator arm model
            robot_model: Robot arm model
            config: Teleoperation configuration
        """
        self.operator_model = operator_model
        self.robot_model = robot_model
        
        # Initialize kinematics
        self.op_kin = Kinematics(self._convert_operator_to_arm_model())
        self.robot_kin = Kinematics(robot_model)
        
        # Configuration
        self.velocity_scale = config.get('velocity_scale', 0.6)
        self.null_space_gain = config.get('mapping', {}).get('null_space_gain', 0.1)
        
        # Damped least squares parameters
        self.lambda2 = 0.01  # Damping factor
        
        # Robot home position
        self.robot_home = np.array([js.home for js in robot_model.joints])
        
    def _convert_operator_to_arm_model(self) -> ArmModel:
        """Convert operator model to ArmModel for kinematics."""
        from ..models import JointSpec, JointLimit
        
        # Create joint specs for operator arm (simplified)
        joints = []
        for i, segment in enumerate(self.operator_model.segments):
            joint_spec = JointSpec(
                name=f"operator_joint_{i}",
                home=0.0,
                limit=JointLimit(min=-np.pi, max=np.pi),
                max_vel=np.deg2rad(180),  # Fast human motion
                max_acc=np.deg2rad(360)
            )
            joints.append(joint_spec)
        
        return ArmModel(joints=tuple(joints), dh=self.operator_model.dh)
    
    def map_velocity(self, op_q: np.ndarray, op_dq: np.ndarray, robot_q: np.ndarray) -> np.ndarray:
        """
        Map operator joint velocities to robot joint velocities.
        
        Args:
            op_q: Operator joint angles (3 DOF)
            op_dq: Operator joint velocities (3 DOF)
            robot_q: Current robot joint angles (5 DOF)
            
        Returns:
            Robot joint velocities (5 DOF)
        """
        # 1. Compute operator end-effector velocity
        J_op = self.op_kin.jacobian_geometric(op_q)
        v_op_ee = J_op @ op_dq  # 6D twist
        
        # 2. Scale velocity
        v_robot_ee = self.velocity_scale * v_op_ee
        
        # 3. Compute robot joint velocities via inverse Jacobian
        J_robot = self.robot_kin.jacobian_geometric(robot_q)
        robot_dq = self._solve_ik_velocity(J_robot, v_robot_ee, robot_q)
        
        return robot_dq
    
    def _solve_ik_velocity(self, J: np.ndarray, v_desired: np.ndarray, q_current: np.ndarray) -> np.ndarray:
        """
        Solve inverse kinematics for velocity using damped least squares.
        
        Args:
            J: Robot Jacobian matrix (6x5)
            v_desired: Desired end-effector velocity (6D)
            q_current: Current robot joint angles (5D)
            
        Returns:
            Robot joint velocities (5D)
        """
        # Damped least squares solution
        JJT = J @ J.T + self.lambda2 * np.eye(6)
        dq_primary = J.T @ np.linalg.solve(JJT, v_desired)
        
        # Null space bias toward home
        dq_null = self.null_space_gain * (self.robot_home - q_current)
        
        # Compute null space projector
        try:
            J_pinv = np.linalg.pinv(J)
            N = np.eye(5) - J_pinv @ J  # Null space projector
        except np.linalg.LinAlgError:
            # Fallback if pseudoinverse fails
            N = np.zeros((5, 5))
        
        # Combine primary and null space velocities
        dq = dq_primary + N @ dq_null
        
        return dq
    
    def get_operator_end_effector_velocity(self, op_q: np.ndarray, op_dq: np.ndarray) -> np.ndarray:
        """Get operator end-effector velocity."""
        J_op = self.op_kin.jacobian_geometric(op_q)
        return J_op @ op_dq
    
    def get_robot_jacobian(self, robot_q: np.ndarray) -> np.ndarray:
        """Get robot Jacobian matrix."""
        return self.robot_kin.jacobian_geometric(robot_q)
    
    def get_manipulability(self, robot_q: np.ndarray) -> float:
        """
        Calculate manipulability measure for robot configuration.
        
        Returns:
            Manipulability measure (higher = better dexterity)
        """
        J = self.robot_kin.jacobian_geometric(robot_q)
        return np.sqrt(np.linalg.det(J @ J.T))


class CartesianVelocityMapper:
    """Alternative mapper that works directly in Cartesian space."""
    
    def __init__(self, robot_model: ArmModel, config: Dict[str, Any]):
        self.robot_model = robot_model
        self.robot_kin = Kinematics(robot_model)
        self.velocity_scale = config.get('velocity_scale', 0.6)
        self.lambda2 = 0.01
        self.robot_home = np.array([js.home for js in robot_model.joints])
        
    def map_cartesian_velocity(self, v_desired_cart: np.ndarray, robot_q: np.ndarray) -> np.ndarray:
        """
        Map desired Cartesian velocity to robot joint velocities.
        
        Args:
            v_desired_cart: Desired end-effector velocity in Cartesian space (6D)
            robot_q: Current robot joint angles (5D)
            
        Returns:
            Robot joint velocities (5D)
        """
        # Scale velocity
        v_scaled = self.velocity_scale * v_desired_cart
        
        # Solve inverse kinematics for velocity
        J = self.robot_kin.jacobian_geometric(robot_q)
        
        # Damped least squares
        JJT = J @ J.T + self.lambda2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJT, v_scaled)
        
        return dq


def test_velocity_mapper():
    """Test the velocity mapper."""
    from ..models import load_from_config, load_operator_from_config
    from ..sensors import create_mock_operator_pose
    
    print("Testing VelocityMapper...")
    
    # Load models
    robot_model = load_from_config()
    operator_model = load_operator_from_config()
    
    # Configuration
    config = {
        'velocity_scale': 0.6,
        'mapping': {'null_space_gain': 0.1}
    }
    
    # Create mapper
    mapper = VelocityMapper(operator_model, robot_model, config)
    
    # Test data
    op_q = np.array([0.1, 0.2, 0.3])  # Operator joint angles
    op_dq = np.array([0.05, 0.1, 0.15])  # Operator velocities
    robot_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Robot at home
    
    # Test mapping
    robot_dq = mapper.map_velocity(op_q, op_dq, robot_q)
    
    print(f"Operator angles: {op_q}")
    print(f"Operator velocities: {op_dq}")
    print(f"Robot velocities: {robot_dq}")
    
    # Test manipulability
    manipulability = mapper.get_manipulability(robot_q)
    print(f"Robot manipulability: {manipulability:.4f}")
    
    print("VelocityMapper test completed")
    return True


if __name__ == "__main__":
    test_velocity_mapper()
