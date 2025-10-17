"""
Telemanipulator Motion Controller (v0)

Motion controller with trajectory generation and inverse kinematics.
"""
from __future__ import annotations
from typing import Protocol, Optional
import numpy as np
from .models import ArmModel
from .kinematics import Kinematics
from .ik import IK
from .trajectory import Trajectory

# -----------------------------
# Hardware abstraction
# -----------------------------

class ServoDriver(Protocol):
    def move_to(self, idx: int, angle_rad: float, speed: Optional[float]=None) -> None: ...
    def angles(self) -> np.ndarray: ...   # current measured/estimated joint angles (rad)

class MotionController:
    def __init__(self, model: ArmModel, driver: ServoDriver):
        self.model = model
        self.driver = driver
        self.kin = Kinematics(model)
        self.ik = IK(self.kin)
        self.trajectory = Trajectory()

    def go_home(self):
        for i, js in enumerate(self.model.joints):
            self.driver.move_to(i, js.home)

    def move_cartesian(self, q_start: np.ndarray, T_goal: np.ndarray, seconds: float = 2.0, steps: int = 50):
        T0 = self.kin.fk(q_start)
        p0, R0 = T0[:3,3], T0[:3,:3]
        pf, Rf = T_goal[:3,3], T_goal[:3,:3]
        # linear in position, slerp-ish via log/exp incremental (small angle per step)
        pos_traj = self.trajectory.cubic_time_scaling(0.0, seconds, p0, pf)
        q = q_start.copy()
        for k in range(steps+1):
            t = seconds * k/steps
            pt = pos_traj(t)
            # simple orientation interpolation via error toward target
            T = self.kin.fk(q)
            Rt = Rf  # keep fixed for v0
            Tt = np.eye(4); Tt[:3,:3] = Rt; Tt[:3,3] = pt
            q = self.ik.solve(q, Tt)
            # enforce joint limits
            for i, js in enumerate(self.model.joints):
                q[i] = np.clip(q[i], js.limit.min, js.limit.max)
                self.driver.move_to(i, q[i])
        return q
