"""
Telemanipulator Core Models (v0)

Data models for joint specifications, DH parameters, and arm models.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Dict, Any, Optional
import numpy as np
import yaml
import os
from pathlib import Path

# -----------------------------
# Core data models
# -----------------------------

@dataclass(frozen=True)
class JointLimit:
    min: float  # radians
    max: float  # radians

@dataclass(frozen=True)
class JointSpec:
    name: str
    home: float
    limit: JointLimit
    max_vel: float = np.deg2rad(90)  # rad/s
    max_acc: float = np.deg2rad(180) # rad/s^2

@dataclass(frozen=True)
class DH:
    """Standard DH parameters for each joint.
    a: link length, alpha: link twist, d: link offset, theta: joint variable
    For revolute joints, theta is the variable; d is constant.
    """
    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0

@dataclass
class ArmModel:
    joints: Tuple[JointSpec, ...]
    dh: Tuple[DH, ...]

    def n(self) -> int:
        return len(self.joints)

@dataclass
class OperatorSegment:
    name: str
    length: float  # meters
    imu_id: int

@dataclass
class OperatorArmModel:
    segments: Tuple[OperatorSegment, ...]
    dh: Tuple[DH, ...]

    def n(self) -> int:
        return len(self.segments)

# -----------------------------
# Example: define a 5-DOF model (placeholder numbers!)
# -----------------------------

def example_model() -> ArmModel:
    joints = (
        JointSpec("base_yaw",   home=0.0, limit=JointLimit(np.deg2rad(-180), np.deg2rad(180))),
        JointSpec("base_pitch", home=0.0, limit=JointLimit(np.deg2rad(-90),  np.deg2rad(90))),
        JointSpec("elbow",      home=0.0, limit=JointLimit(np.deg2rad(-135), np.deg2rad(135))),
        JointSpec("wrist_pitch",home=0.0, limit=JointLimit(np.deg2rad(-135), np.deg2rad(135))),
        JointSpec("wrist_roll", home=0.0, limit=JointLimit(np.deg2rad(-180), np.deg2rad(180))),
    )
    # Replace with measured DH once assembled
    dh = (
        DH(a=0.05, alpha=np.deg2rad(90),  d=0.10),
        DH(a=0.15, alpha=0.0,             d=0.00),
        DH(a=0.12, alpha=0.0,             d=0.00),
        DH(a=0.05, alpha=np.deg2rad(90),  d=0.00),
        DH(a=0.02, alpha=0.0,             d=0.00),
    )
    return ArmModel(joints=joints, dh=dh)

# -----------------------------
# Configuration loading
# -----------------------------

def load_robot_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Load robot configuration from YAML file.
    
    Args:
        config_path: Path to robot.yaml config file. If None, uses default location.
        
    Returns:
        Dictionary containing robot configuration.
    """
    if config_path is None:
        # Default to config/robot.yaml relative to this package
        package_dir = Path(__file__).parent.parent
        config_path = package_dir / "config" / "robot.yaml"
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Robot config file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def load_from_config(config_path: Optional[str] = None) -> ArmModel:
    """Create ArmModel from YAML configuration file.
    
    Args:
        config_path: Path to robot.yaml config file. If None, uses default location.
        
    Returns:
        ArmModel loaded from configuration.
    """
    config = load_robot_config(config_path)
    
    # Build joint specifications
    joints = []
    for joint_data in config['joints']:
        joint_spec = JointSpec(
            name=joint_data['name'],
            home=np.deg2rad(joint_data['home']),
            limit=JointLimit(
                min=np.deg2rad(joint_data['limit_min']),
                max=np.deg2rad(joint_data['limit_max'])
            ),
            max_vel=np.deg2rad(joint_data.get('max_vel', 90.0)),
            max_acc=np.deg2rad(joint_data.get('max_acc', 180.0))
        )
        joints.append(joint_spec)
    
    # Build DH parameters
    dh_params = []
    for dh_data in config['dh_parameters']['links']:
        dh = DH(
            a=dh_data['a'],
            alpha=np.deg2rad(dh_data['alpha']),
            d=dh_data['d'],
            theta_offset=np.deg2rad(dh_data.get('theta_offset', 0.0))
        )
        dh_params.append(dh)
    
    return ArmModel(joints=tuple(joints), dh=tuple(dh_params))

def load_operator_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Load operator arm configuration from YAML file.
    
    Args:
        config_path: Path to operator_arm.yaml config file. If None, uses default location.
        
    Returns:
        Dictionary containing operator arm configuration.
    """
    if config_path is None:
        # Default to config/operator_arm.yaml relative to this package
        package_dir = Path(__file__).parent.parent
        config_path = package_dir / "config" / "operator_arm.yaml"
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Operator config file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def load_operator_from_config(config_path: Optional[str] = None) -> OperatorArmModel:
    """Create OperatorArmModel from YAML configuration file.
    
    Args:
        config_path: Path to operator_arm.yaml config file. If None, uses default location.
        
    Returns:
        OperatorArmModel loaded from configuration.
    """
    config = load_operator_config(config_path)
    
    # Build segment specifications
    segments = []
    for segment_data in config['segments']:
        segment = OperatorSegment(
            name=segment_data['name'],
            length=segment_data['length'],
            imu_id=segment_data['imu_id']
        )
        segments.append(segment)
    
    # Build DH parameters
    dh_params = []
    for dh_data in config['dh_parameters']['links']:
        dh = DH(
            a=dh_data['a'],
            alpha=np.deg2rad(dh_data['alpha']),
            d=dh_data['d'],
            theta_offset=np.deg2rad(dh_data.get('theta_offset', 0.0))
        )
        dh_params.append(dh)
    
    return OperatorArmModel(segments=tuple(segments), dh=tuple(dh_params))
