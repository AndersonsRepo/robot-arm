"""
Telemanipulator Core Models (v0)

Data models for joint specifications, DH parameters, and arm models.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple
import numpy as np

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
