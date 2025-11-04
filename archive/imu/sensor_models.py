"""
IMU Sensor Data Structures for Telemanipulation (Archived)

Data models for IMU readings and operator pose estimation.
Archived as part of migration from IMU-based teleoperation to gamepad-based control.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class IMUReading:
    """Raw IMU sensor reading from MPU-9250."""
    timestamp: float  # seconds
    accel: np.ndarray  # 3D acceleration (m/s^2)
    gyro: np.ndarray   # 3D angular velocity (rad/s)
    mag: np.ndarray    # 3D magnetometer (optional, μT)
    imu_id: int        # Which IMU (0=upper_arm, 1=forearm, 2=hand)
    
    def __post_init__(self):
        # Ensure arrays are 3D
        self.accel = np.array(self.accel, dtype=float)
        self.gyro = np.array(self.gyro, dtype=float)
        self.mag = np.array(self.mag, dtype=float)
        
        assert len(self.accel) == 3, "Acceleration must be 3D"
        assert len(self.gyro) == 3, "Angular velocity must be 3D"
        assert len(self.mag) == 3, "Magnetometer must be 3D"
        assert 0 <= self.imu_id <= 2, "IMU ID must be 0, 1, or 2"


@dataclass
class Orientation:
    """Estimated orientation from IMU fusion."""
    timestamp: float
    quaternion: np.ndarray  # [w, x, y, z] quaternion
    euler_angles: np.ndarray  # [roll, pitch, yaw] in radians
    imu_id: int
    confidence: float  # Fusion confidence 0-1
    
    def __post_init__(self):
        self.quaternion = np.array(self.quaternion, dtype=float)
        self.euler_angles = np.array(self.euler_angles, dtype=float)
        
        assert len(self.quaternion) == 4, "Quaternion must be 4D"
        assert len(self.euler_angles) == 3, "Euler angles must be 3D"
        assert 0 <= self.confidence <= 1, "Confidence must be 0-1"


@dataclass
class OperatorPose:
    """Complete operator arm pose from IMU fusion."""
    timestamp: float
    joint_angles: np.ndarray  # 3 DOF joint angles (radians)
    joint_velocities: np.ndarray  # 3 DOF joint velocities (rad/s)
    confidence: float  # Overall fusion confidence 0-1
    orientations: List[Orientation]  # Individual IMU orientations
    
    def __post_init__(self):
        self.joint_angles = np.array(self.joint_angles, dtype=float)
        self.joint_velocities = np.array(self.joint_velocities, dtype=float)
        
        assert len(self.joint_angles) == 3, "Joint angles must be 3D"
        assert len(self.joint_velocities) == 3, "Joint velocities must be 3D"
        assert 0 <= self.confidence <= 1, "Confidence must be 0-1"


def create_mock_imu_reading(imu_id: int, timestamp: float = None) -> IMUReading:
    """Create mock IMU reading for testing."""
    import time
    
    if timestamp is None:
        timestamp = time.time()
    
    # Generate realistic sensor noise
    accel_noise = np.random.normal(0, 0.1, 3)
    gyro_noise = np.random.normal(0, 0.01, 3)
    mag_noise = np.random.normal(0, 1.0, 3)
    
    # Base readings (gravity + small motion)
    base_accel = np.array([0, 0, -9.81])  # gravity
    base_gyro = np.array([0.1, 0.05, 0.02])  # small rotation
    base_mag = np.array([20, 5, -45])  # Earth's magnetic field
    
    return IMUReading(
        timestamp=timestamp,
        accel=base_accel + accel_noise,
        gyro=base_gyro + gyro_noise,
        mag=base_mag + mag_noise,
        imu_id=imu_id
    )


def create_mock_operator_pose(timestamp: float = None) -> OperatorPose:
    """Create mock operator pose for testing."""
    import time
    
    if timestamp is None:
        timestamp = time.time()
    
    # Generate realistic joint angles (small movements)
    joint_angles = np.array([
        np.random.uniform(-0.5, 0.5),  # shoulder
        np.random.uniform(-1.0, 0.5),  # elbow
        np.random.uniform(-0.3, 0.3)   # wrist
    ])
    
    # Generate realistic velocities
    joint_velocities = np.random.uniform(-0.5, 0.5, 3)
    
    # Create mock orientations for each IMU
    orientations = []
    for imu_id in range(3):
        orientation = Orientation(
            timestamp=timestamp,
            quaternion=np.array([1, 0, 0, 0]),  # identity quaternion
            euler_angles=np.array([0, 0, 0]),
            imu_id=imu_id,
            confidence=0.9
        )
        orientations.append(orientation)
    
    return OperatorPose(
        timestamp=timestamp,
        joint_angles=joint_angles,
        joint_velocities=joint_velocities,
        confidence=0.85,
        orientations=orientations
    )

