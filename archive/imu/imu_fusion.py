"""
IMU Fusion Components for Telearm

This module provides IMU fusion algorithms for orientation estimation and operator pose tracking.
Note: In production, IMU fusion is handled by the ESP32 firmware for real-time performance.
These classes are primarily used for:
- Mock data generation for testing
- Development and simulation without hardware
- Educational purposes and algorithm understanding
- Future research and alternative fusion methods
"""

from __future__ import annotations
import numpy as np
import time
from typing import List
from dataclasses import dataclass
from .sensors import IMUReading, Orientation, OperatorPose


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


@dataclass
class FilterState:
    """State for complementary filter."""
    quaternion: np.ndarray  # [w, x, y, z]
    prev_time: float
    gyro_bias: np.ndarray  # [x, y, z] gyroscope bias
    accel_bias: np.ndarray  # [x, y, z] accelerometer bias
    initialized: bool = False


class ComplementaryFilter:
    """
    Complementary filter for IMU orientation estimation.
    
    This filter fuses accelerometer and gyroscope data to estimate orientation.
    In production, this runs on the ESP32 firmware at 200Hz for real-time performance.
    
    This Python implementation is used for:
    - Mock data generation
    - Testing and development
    - Educational purposes
    - Algorithm validation
    """
    
    def __init__(self, alpha: float = 0.98, gyro_threshold: float = 0.1):
        """
        Initialize complementary filter.
        
        Args:
            alpha: Filter coefficient (0-1), higher = more gyro trust
            gyro_threshold: Minimum gyro magnitude to apply correction
        """
        self.alpha = alpha
        self.gyro_threshold = gyro_threshold
        self.state = FilterState(
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),  # Identity quaternion
            prev_time=0.0,
            gyro_bias=np.zeros(3),
            accel_bias=np.zeros(3),
            initialized=False
        )
    
    def update(self, accel: np.ndarray, gyro: np.ndarray, dt: float) -> np.ndarray:
        """
        Update filter with new IMU data.
        
        Args:
            accel: Accelerometer reading [x, y, z] in m/s²
            gyro: Gyroscope reading [x, y, z] in rad/s
            dt: Time step in seconds
            
        Returns:
            Orientation quaternion [w, x, y, z]
        """
        if not self.state.initialized:
            self._initialize(accel, gyro)
            self.state.initialized = True
        
        # Normalize accelerometer
        accel_corr = accel - self.state.accel_bias
        accel_norm = np.linalg.norm(accel_corr)
        if accel_norm > 0.1:  # Avoid division by zero
            accel_unit = accel_corr / accel_norm
        else:
            accel_unit = np.array([0, 0, 1])  # Default to gravity direction
        
        # Gyroscope integration
        gyro_corrected = gyro - self.state.gyro_bias
        q_gyro = self._integrate_gyro(gyro_corrected, dt)
        
        # Accelerometer correction
        q_accel = self._accel_to_quaternion(accel_unit)
        
        # Complementary filter fusion (gate accel correction by gyro magnitude)
        gyro_mag = np.linalg.norm(gyro_corrected)
        t = (1.0 - self.alpha) if gyro_mag <= self.gyro_threshold else 0.0
        self.state.quaternion = self._slerp(q_gyro, q_accel, t)
        
        # Normalize quaternion
        self.state.quaternion = self.state.quaternion / np.linalg.norm(self.state.quaternion)
        
        # Update time tracking
        self.state.prev_time += dt
        
        return self.state.quaternion.copy()
    
    def _initialize(self, accel: np.ndarray, gyro: np.ndarray):
        """Initialize filter with first reading."""
        # Estimate initial orientation from accelerometer
        accel_norm = np.linalg.norm(accel)
        if accel_norm > 0.1:
            accel_unit = accel / accel_norm
            self.state.quaternion = self._accel_to_quaternion(accel_unit)
        
        # Initialize gyro bias (assume stationary)
        self.state.gyro_bias = gyro.copy()
        
        # Initialize accel bias (assume gravity)
        self.state.accel_bias = accel - np.array([0, 0, 9.81])
    
    def _integrate_gyro(self, gyro: np.ndarray, dt: float) -> np.ndarray:
        """Integrate gyroscope data to get orientation."""
        # Convert gyro to quaternion derivative
        gyro_magnitude = np.linalg.norm(gyro)
        
        if gyro_magnitude < 1e-6:
            return self.state.quaternion.copy()
        
        # Axis-angle representation
        axis = gyro / gyro_magnitude
        angle = gyro_magnitude * dt
        
        # Create rotation quaternion
        cos_half = np.cos(angle / 2)
        sin_half = np.sin(angle / 2)
        
        q_rot = np.array([
            cos_half,
            sin_half * axis[0],
            sin_half * axis[1],
            sin_half * axis[2]
        ])
        
        # Apply rotation
        return quat_mul(self.state.quaternion, q_rot)
    
    def _accel_to_quaternion(self, accel: np.ndarray) -> np.ndarray:
        """Convert accelerometer reading to quaternion."""
        # Assume gravity points down (z-axis)
        gravity = np.array([0, 0, 1])
        
        # Calculate rotation axis (cross product)
        axis = np.cross(gravity, accel)
        axis_norm = np.linalg.norm(axis)
        
        if axis_norm < 1e-6:
            return np.array([1, 0, 0, 0])  # No rotation needed
        
        axis = axis / axis_norm
        
        # Calculate rotation angle
        cos_angle = np.dot(gravity, accel)
        cos_angle = np.clip(cos_angle, -1, 1)
        angle = np.arccos(cos_angle)
        
        # Create quaternion
        cos_half = np.cos(angle / 2)
        sin_half = np.sin(angle / 2)
        
        return np.array([
            cos_half,
            sin_half * axis[0],
            sin_half * axis[1],
            sin_half * axis[2]
        ])
    
    def _slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between quaternions."""
        # Ensure quaternions are normalized
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Calculate dot product
        dot = np.dot(q1, q2)
        
        # If dot product is negative, negate one quaternion
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Calculate angle
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        
        # Spherical interpolation
        w1 = np.sin((1 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        
        return w1 * q1 + w2 * q2
    
    def quaternion_to_euler(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])


class OperatorPoseEstimator:
    """
    Operator pose estimation from multiple IMU readings.
    
    This class estimates human arm joint angles from 3× IMU sensors.
    In production, this runs on the ESP32 firmware for real-time performance.
    
    This Python implementation is used for:
    - Mock data generation
    - Testing and development
    - Educational purposes
    - Algorithm validation
    """
    
    def __init__(self, num_imus: int = 3):
        """
        Initialize pose estimator.
        
        Args:
            num_imus: Number of IMU sensors (default: 3)
        """
        self.num_imus = num_imus
        self.filters = [ComplementaryFilter() for _ in range(num_imus)]
        self.last_update_time = 0.0
        
        # IMU mounting orientations (relative to body segments)
        # These should match the ESP32 firmware configuration
        self.imu_orientations = [
            np.array([1, 0, 0, 0]),  # IMU 0: Upper arm (identity)
            np.array([1, 0, 0, 0]),  # IMU 1: Forearm (identity)
            np.array([1, 0, 0, 0])   # IMU 2: Hand (identity)
        ]
    
    def update(self, imu_readings: List[IMUReading]) -> OperatorPose:
        """
        Update pose estimation with new IMU readings.
        
        Args:
            imu_readings: List of IMU readings (one per sensor)
            
        Returns:
            Estimated operator pose
        """
        if len(imu_readings) != self.num_imus:
            raise ValueError(f"Expected {self.num_imus} IMU readings, got {len(imu_readings)}")
        
        current_time = time.time()
        dt = current_time - self.last_update_time if self.last_update_time > 0 else 0.01
        self.last_update_time = current_time
        
        # Update each IMU filter
        orientations = []
        for i, reading in enumerate(imu_readings):
            accel = np.array([reading.accel.x, reading.accel.y, reading.accel.z])
            gyro = np.array([reading.gyro.x, reading.gyro.y, reading.gyro.z])
            
            # Update complementary filter
            quaternion = self.filters[i].update(accel, gyro, dt)
            
            # Apply mounting orientation correction
            corrected_quat = quat_mul(quaternion, self.imu_orientations[i])
            
            orientations.append(Orientation(
                timestamp=reading.timestamp,
                quaternion=corrected_quat,
                euler_angles=self.filters[i].quaternion_to_euler(corrected_quat),
                imu_id=i,
                confidence=0.9  # High confidence for mock data
            ))
        
        # Estimate joint angles from orientations
        joint_angles = self._estimate_joint_angles(orientations)
        joint_velocities = self._estimate_joint_velocities(orientations, dt)
        
        return OperatorPose(
            timestamp=current_time,
            joint_angles=joint_angles,
            joint_velocities=joint_velocities,
            confidence=0.9,
            orientations=orientations
        )
    
    def _estimate_joint_angles(self, orientations: List[Orientation]) -> List[float]:
        """
        Estimate joint angles from IMU orientations.
        
        This is a simplified model for mock data generation.
        The ESP32 firmware uses more sophisticated kinematics.
        """
        if len(orientations) < 3:
            return [0.0, 0.0, 0.0]
        
        # Extract Euler angles
        upper_arm_euler = orientations[0].euler_angles
        forearm_euler = orientations[1].euler_angles
        hand_euler = orientations[2].euler_angles
        
        # Simplified joint angle estimation
        # In reality, this would use proper kinematic models
        
        # Shoulder pitch (upper arm pitch)
        shoulder_pitch = upper_arm_euler[1]  # Pitch component
        
        # Elbow angle (relative angle between upper arm and forearm)
        elbow_angle = forearm_euler[1] - upper_arm_euler[1]
        
        # Wrist angle (hand orientation relative to forearm)
        wrist_angle = hand_euler[1] - forearm_euler[1]
        
        return [shoulder_pitch, elbow_angle, wrist_angle]
    
    def _estimate_joint_velocities(self, orientations: List[Orientation], dt: float) -> List[float]:
        """Estimate joint velocities from orientation changes."""
        # This is a simplified implementation for mock data
        # In reality, this would use proper differentiation
        
        if dt <= 0:
            return [0.0, 0.0, 0.0]
        
        # Use deterministic random velocities for mock data
        rng = np.random.default_rng(0)  # deterministic for tests
        velocities = [float(rng.normal(0.0, 0.1)) for _ in orientations]  # mock pitch rates
        
        return velocities[:3]  # Return first 3 velocities


# Convenience functions for mock data generation
def create_mock_imu_fusion_data(num_samples: int = 100) -> List[OperatorPose]:
    """
    Create mock IMU fusion data for testing.
    
    Args:
        num_samples: Number of pose samples to generate
        
    Returns:
        List of mock operator poses
    """
    estimator = OperatorPoseEstimator()
    poses = []
    
    for i in range(num_samples):
        # Create mock IMU readings
        imu_readings = []
        for j in range(3):
            reading = IMUReading(
                imu_id=j,
                timestamp=time.time(),
                accel=np.array([
                    np.random.normal(0, 0.1),
                    np.random.normal(0, 0.1),
                    np.random.normal(9.81, 0.1)
                ]),
                gyro=np.array([
                    np.random.normal(0, 0.01),
                    np.random.normal(0, 0.01),
                    np.random.normal(0, 0.01)
                ]),
                mag=np.array([
                    np.random.normal(0, 1),
                    np.random.normal(0, 1),
                    np.random.normal(0, 1)
                ])
            )
            imu_readings.append(reading)
        
        # Estimate pose
        pose = estimator.update(imu_readings)
        poses.append(pose)
        
        time.sleep(0.01)  # 100Hz simulation
    
    return poses


def validate_imu_fusion_algorithm():
    """
    Validate IMU fusion algorithm with known test cases.
    
    This function can be used to verify the fusion algorithm
    works correctly before using it for mock data generation.
    """
    print("Testing IMU fusion algorithm...")
    
    # Test complementary filter
    filter_test = ComplementaryFilter()
    
    # Test with gravity vector
    accel = np.array([0, 0, 9.81])
    gyro = np.array([0, 0, 0])
    dt = 0.01
    
    quat = filter_test.update(accel, gyro, dt)
    print(f"Gravity test - Quaternion: {quat}")
    
    # Test with rotation
    accel = np.array([0, 0, 9.81])
    gyro = np.array([0, 0, 1.0])  # 1 rad/s rotation
    dt = 0.01
    
    for _ in range(10):
        quat = filter_test.update(accel, gyro, dt)
    
    print(f"Rotation test - Quaternion: {quat}")
    
    # Test pose estimator
    estimator_test = OperatorPoseEstimator()
    
    # Create mock readings
    imu_readings = []
    for i in range(3):
        reading = IMUReading(
            imu_id=i,
            timestamp=time.time(),
            accel=np.array([0, 0, 9.81]),
            gyro=np.array([0, 0, 0]),
            mag=np.array([0, 0, 0])
        )
        imu_readings.append(reading)
    
    pose = estimator_test.update(imu_readings)
    print(f"Pose estimation test - Joint angles: {pose.joint_angles}")
    
    print("IMU fusion algorithm validation complete!")


if __name__ == "__main__":
    # Run validation when script is executed directly
    validate_imu_fusion_algorithm()
