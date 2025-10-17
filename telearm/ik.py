"""
Telemanipulator Inverse Kinematics (v0)

Damped least squares inverse kinematics solver.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .kinematics import Kinematics

@dataclass
class IKOptions:
    max_iters: int = 200
    tol_pos: float = 1e-3
    tol_rot: float = 1e-2
    lambda2: float = 1e-4  # damping^2

class IK:
    def __init__(self, kin: Kinematics):
        self.kin = kin

    @staticmethod
    def _log_SO3(R: np.ndarray) -> np.ndarray:
        """Axis-angle vector from rotation matrix (vee(log(R)))."""
        cos_theta = (np.trace(R) - 1)/2
        cos_theta = np.clip(cos_theta, -1, 1)
        theta = np.arccos(cos_theta)
        if theta < 1e-9:
            return np.zeros(3)
        w = (1/(2*np.sin(theta))) * np.array([
            R[2,1] - R[1,2],
            R[0,2] - R[2,0],
            R[1,0] - R[0,1],
        ])
        return theta * w

    def solve(self, q0: np.ndarray, T_target: np.ndarray, opts: IKOptions = IKOptions()) -> np.ndarray:
        q = q0.copy()
        for _ in range(opts.max_iters):
            T = self.kin.fk(q)
            p, R = T[:3,3], T[:3,:3]
            pt, Rt = T_target[:3,3], T_target[:3,:3]
            ep = pt - p
            eR = self._log_SO3(R.T @ Rt)
            e = np.concatenate([ep, eR])
            if np.linalg.norm(ep) < opts.tol_pos and np.linalg.norm(eR) < opts.tol_rot:
                break
            J = self.kin.jacobian_geometric(q)
            JJ = J @ J.T + opts.lambda2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJ, e)
            q = q + dq
        return q
