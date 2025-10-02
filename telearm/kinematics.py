"""
Telemanipulator Kinematics (v0)

Forward kinematics and Jacobian computation using DH parameters.
"""
from __future__ import annotations
from typing import Sequence
import numpy as np
from .models import ArmModel

class Kinematics:
    def __init__(self, model: ArmModel):
        assert model.n() == len(model.dh)
        self.m = model

    @staticmethod
    def _dh_mat(a, alpha, d, theta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ ct, -st*ca,  st*sa, a*ct],
            [ st,  ct*ca, -ct*sa, a*st],
            [  0,     sa,     ca,    d],
            [  0,      0,      0,    1],
        ])

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """Forward kinematics: returns 4x4 pose of end-effector in base frame."""
        T = np.eye(4)
        for qi, dh in zip(q, self.m.dh):
            T = T @ self._dh_mat(dh.a, dh.alpha, dh.d, qi + dh.theta_offset)
        return T

    def jacobian_geometric(self, q: Sequence[float]) -> np.ndarray:
        """Geometric Jacobian (6xn) for a chain of revolute joints."""
        n = self.m.n()
        T = np.eye(4)
        origins = [T[:3, 3]]
        z_axes  = [T[:3, 2]]
        Ts = [T]
        # forward pass, store frames
        for qi, dh in zip(q, self.m.dh):
            T = T @ self._dh_mat(dh.a, dh.alpha, dh.d, qi + dh.theta_offset)
            Ts.append(T)
            origins.append(T[:3, 3])
            z_axes.append(T[:3, 2])
        J = np.zeros((6, n))
        pe = origins[-1]
        for i in range(n):
            zi = z_axes[i]
            pi = origins[i]
            Jv = np.cross(zi, pe - pi)
            Jw = zi
            J[:3, i] = Jv
            J[3:, i] = Jw
        return J
