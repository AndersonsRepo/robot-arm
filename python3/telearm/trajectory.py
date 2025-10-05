"""
Telemanipulator Trajectory Generation (v0)

Cubic time scaling between waypoints.
"""
from __future__ import annotations
from typing import Callable
import numpy as np

class Trajectory:
    @staticmethod
    def cubic_time_scaling(t0: float, tf: float, p0: np.ndarray, pf: np.ndarray) -> Callable[[float], np.ndarray]:
        dt = tf - t0
        if dt <= 0: raise ValueError("tf must be greater than t0")
        a0 = p0
        a3 = 2*(p0 - pf) / dt**3
        a2 = 3*(pf - p0) / dt**2
        def s(t: float) -> np.ndarray:
            tau = np.clip((t - t0)/dt, 0.0, 1.0)
            return a0 + a2*(tau*dt)**2 + a3*(tau*dt)**3
        return s
