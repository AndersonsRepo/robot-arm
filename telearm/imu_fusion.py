"""
IMU Fusion Algorithms for Telemanipulation

Complementary filter and Kalman filter implementations for fusing IMU data
from multiple sensors to estimate operator arm pose.
"""
from __future__ import annotations
from typing import List, Tuple
import numpy as np
import time
from .sensors import IMUReading, Orientation, OperatorPose
from .models import OperatorArmModel


class ComplementaryFilter:
    """Complementary filter for IMU orientation estimation."""
    
    def __init__(self, alpha: float = 0.98):
        """
        Args:
            alpha: Filter coefficient (0-1). Higher = more trust in gyro, lower = more trust in accel/mag.
        """
        self.alpha = alpha
        self.prev_time = None
        self.prev_quat = np.array([1, 0, 0, 0])  # [w, x, y, z]
        
    def update(self, reading: IMUReading) -> Orientation:
        """Update filter with new IMU reading."""
        current_time = reading.timestamp
        
        # Initialize on first reading
        if self.prev_time is None:
            self.prev_time = current_time
            # Initialize with accelerometer tilt
            self.prev_quat = self._accel_to_quat(reading.accel)
            return Orientation(
                timestamp=current_time,
                quaternion=self.prev_quat.copy(),
                euler_angles=self._quat_to_euler(self.prev_quat),
                imu_id=reading.imu_id,
                confidence=0.5
            )
        
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.001  # Minimum time step
        
        # Gyro integration
        gyro_quat = self._gyro_to_quat(reading.gyro, dt)
        quat_gyro = self._quat_multiply(self.prev_quat, gyro_quat)
        
        # Accelerometer correction (tilt only)
        accel_quat = self._accel_to_quat(reading.accel)
        
        # Complementary filter
        quat_filtered = self._slerp(accel_quat, quat_gyro, self.alpha)
        
        # Normalize quaternion
        quat_filtered = quat_filtered / np.linalg.norm(quat_filtered)
        
        self.prev_quat = quat_filtered
        self.prev_time = current_time
        
        return Orientation(
            timestamp=current_time,
            quaternion=quat_filtered.copy(),
            euler_angles=self._quat_to_euler(quat_filtered),
            imu_id=reading.imu_id,
            confidence=0.9
        )
    
    def _accel_to_quat(self, accel: np.ndarray) -> np.ndarray:
        """Convert accelerometer reading to quaternion (tilt only)."""
        # Normalize accelerometer
        accel_norm = accel / np.linalg.norm(accel)
        
        # Calculate tilt angles
        roll = np.arctan2(accel_norm[1], accel_norm[2])
        pitch = np.arctan2(-accel_norm[0], np.sqrt(accel_norm[1]**2 + accel_norm[2]**2))
        
        # Convert to quaternion
        cr = np.cos(roll / 2)
        sr = np.sin(roll / 2)
        cp = np.cos(pitch / 2)
        sp = np.sin(pitch / 2)
        
        return np.array([
            cr * cp,  # w
            sr * cp,  # x
            cr * sp,  # y
            -sr * sp  # z
        ])
    
    def _gyro_to_quat(self, gyro: np.ndarray, dt: float) -> np.ndarray:
        """Convert gyroscope reading to quaternion delta."""
        # Angular velocity magnitude
        omega = np.linalg.norm(gyro)
        
        if omega < 1e-6:  # No rotation
            return np.array([1, 0, 0, 0])
        
        # Axis of rotation
        axis = gyro / omega
        
        # Quaternion from axis-angle
        half_angle = omega * dt / 2
        s = np.sin(half_angle)
        c = np.cos(half_angle)
        
        return np.array([
            c,           # w
            axis[0] * s, # x
            axis[1] * s, # y
            axis[2] * s  # z
        ])
    
    def _quat_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def _slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between quaternions."""
        # Ensure quaternions are normalized
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Dot product
        dot = np.dot(q1, q2)
        
        # If dot product is negative, slerp won't take the shorter path
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        # If the inputs are too close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Calculate angle between quaternions
        theta_0 = np.arccos(abs(dot))
        sin_theta_0 = np.sin(theta_0)
        
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2
    
    def _quat_to_euler(self, quat: np.ndarray) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = quat
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])


class OperatorPoseEstimator:
    """Estimates operator arm pose from multiple IMU readings."""
    
    def __init__(self, operator_model: OperatorArmModel):
        self.operator_model = operator_model
        self.filters = [ComplementaryFilter() for _ in range(3)]  # One filter per IMU
        self.prev_joint_angles = np.zeros(3)
        self.prev_time = None
        
    def fuse_imu_data(self, imu_readings: List[IMUReading]) -> OperatorPose:
        """Fuse multiple IMU readings to estimate operator pose."""
        if len(imu_readings) != 3:
            raise ValueError(f"Expected 3 IMU readings, got {len(imu_readings)}")
        
        # Sort by IMU ID to ensure consistent ordering
        imu_readings.sort(key=lambda r: r.imu_id)
        
        # Update each filter
        orientations = []
        for i, reading in enumerate(imu_readings):
            if reading.imu_id != i:
                raise ValueError(f"Expected IMU ID {i}, got {reading.imu_id}")
            
            orientation = self.filters[i].update(reading)
            orientations.append(orientation)
        
        # Estimate joint angles from orientations
        joint_angles = self._estimate_joint_angles(orientations)
        
        # Estimate joint velocities
        current_time = time.time()
        joint_velocities = self._estimate_joint_velocities(joint_angles, current_time)
        
        # Calculate overall confidence
        confidence = self._calculate_confidence(orientations)
        
        return OperatorPose(
            timestamp=current_time,
            joint_angles=joint_angles,
            joint_velocities=joint_velocities,
            confidence=confidence,
            orientations=orientations
        )
    
    def _estimate_joint_angles(self, orientations: List[Orientation]) -> np.ndarray:
        """Estimate joint angles from IMU orientations."""
        # This is a simplified approach - in practice, you'd use more sophisticated
        # kinematic analysis to relate IMU orientations to joint angles
        
        joint_angles = np.zeros(3)
        
        # Upper arm orientation (IMU 0) - shoulder pitch
        upper_arm_quat = orientations[0].quaternion
        upper_arm_euler = orientations[0].euler_angles
        joint_angles[0] = upper_arm_euler[1]  # pitch component
        
        # Forearm orientation (IMU 1) relative to upper arm - elbow angle
        forearm_quat = orientations[1].quaternion
        forearm_euler = orientations[1].euler_angles
        
        # Elbow angle is the relative rotation between upper arm and forearm
        # Simplified: use pitch difference
        joint_angles[1] = forearm_euler[1] - upper_arm_euler[1]
        
        # Hand orientation (IMU 2) relative to forearm - wrist angle
        hand_quat = orientations[2].quaternion
        hand_euler = orientations[2].euler_angles
        joint_angles[2] = hand_euler[1] - forearm_euler[1]  # wrist pitch
        
        # Clamp to reasonable ranges
        joint_angles[0] = np.clip(joint_angles[0], -np.pi/2, np.pi/2)  # shoulder
        joint_angles[1] = np.clip(joint_angles[1], -np.pi, np.pi)      # elbow
        joint_angles[2] = np.clip(joint_angles[2], -np.pi/2, np.pi/2)  # wrist
        
        return joint_angles
    
    def _estimate_joint_velocities(self, joint_angles: np.ndarray, current_time: float) -> np.ndarray:
        """Estimate joint velocities from joint angle changes."""
        if self.prev_time is None:
            self.prev_joint_angles = joint_angles.copy()
            self.prev_time = current_time
            return np.zeros(3)
        
        dt = current_time - self.prev_time
        if dt <= 0:
            return np.zeros(3)
        
        # Calculate velocity as finite difference
        joint_velocities = (joint_angles - self.prev_joint_angles) / dt
        
        # Simple low-pass filter to reduce noise
        alpha = 0.7  # Filter coefficient
        if hasattr(self, 'prev_velocities'):
            joint_velocities = alpha * joint_velocities + (1 - alpha) * self.prev_velocities
        
        self.prev_velocities = joint_velocities.copy()
        self.prev_joint_angles = joint_angles.copy()
        self.prev_time = current_time
        
        return joint_velocities
    
    def _calculate_confidence(self, orientations: List[Orientation]) -> float:
        """Calculate overall confidence based on individual IMU confidences."""
        confidences = [o.confidence for o in orientations]
        return np.mean(confidences)


def create_mock_imu_readings(timestamp: float = None) -> List[IMUReading]:
    """Create mock IMU readings for testing."""
    from .sensors import create_mock_imu_reading
    
    if timestamp is None:
        timestamp = time.time()
    
    readings = []
    for imu_id in range(3):
        reading = create_mock_imu_reading(imu_id, timestamp)
        readings.append(reading)
    
    return readings


def test_imu_fusion():
    """Test IMU fusion with mock data."""
    from .models import load_operator_from_config
    
    # Load operator model
    operator_model = load_operator_from_config()
    
    # Create estimator
    estimator = OperatorPoseEstimator(operator_model)
    
    # Test with mock data
    readings = create_mock_imu_readings()
    pose = estimator.fuse_imu_data(readings)
    
    print(f"Estimated pose:")
    print(f"  Joint angles: {pose.joint_angles}")
    print(f"  Joint velocities: {pose.joint_velocities}")
    print(f"  Confidence: {pose.confidence}")
    
    return pose


if __name__ == "__main__":
    test_imu_fusion()
