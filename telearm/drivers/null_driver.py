"""
Null Servo Driver (v0)

Simulation-only placeholder that just stores target angles.
"""
from __future__ import annotations
from typing import Optional
import numpy as np

class NullServoDriver:
    """Sim-only placeholder that just stores target angles."""
    def __init__(self, n: int):
        self._q = np.zeros(n)
    def move_to(self, idx: int, angle_rad: float, speed: Optional[float]=None) -> None:
        self._q[idx] = angle_rad
    def angles(self) -> np.ndarray:
        return self._q.copy()
