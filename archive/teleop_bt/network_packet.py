"""
Network Packet for Teleoperation (Archived)

Network packet format for teleoperation data over WiFi/Bluetooth.
Archived as part of migration from network teleoperation to gamepad-based control.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class TeleopPacket:
    """Network packet for teleoperation data."""
    sequence: int
    timestamp: float
    operator_joints: np.ndarray  # 3 DOF
    operator_velocities: np.ndarray  # 3 DOF
    confidence: float
    
    def __post_init__(self):
        self.operator_joints = np.array(self.operator_joints, dtype=float)
        self.operator_velocities = np.array(self.operator_velocities, dtype=float)
        
        assert len(self.operator_joints) == 3, "Operator joints must be 3D"
        assert len(self.operator_velocities) == 3, "Operator velocities must be 3D"
        assert 0 <= self.confidence <= 1, "Confidence must be 0-1"
    
    def pack(self) -> bytes:
        """Pack to 40-byte UDP packet."""
        import struct
        
        # Pack: seq(4) + timestamp(4) + joints(12) + velocities(12) + confidence(4) + checksum(4)
        data = struct.pack('>I', self.sequence)  # 4 bytes, big-endian
        data += struct.pack('>f', self.timestamp)  # 4 bytes
        
        # Pack joint angles (3 * 4 bytes = 12 bytes)
        for angle in self.operator_joints:
            data += struct.pack('>f', angle)
            
        # Pack velocities (3 * 4 bytes = 12 bytes)
        for vel in self.operator_velocities:
            data += struct.pack('>f', vel)
            
        data += struct.pack('>f', self.confidence)  # 4 bytes
        
        # Calculate simple checksum (sum of all bytes)
        checksum = sum(data) & 0xFFFFFFFF
        data += struct.pack('>I', checksum)  # 4 bytes
        
        assert len(data) == 40, f"Packet must be 40 bytes, got {len(data)}"
        return data
    
    @staticmethod
    def unpack(data: bytes) -> 'TeleopPacket':
        """Unpack from 40-byte UDP packet."""
        import struct
        
        if len(data) != 40:
            raise ValueError(f"Packet must be 40 bytes, got {len(data)}")
        
        # Unpack header
        sequence = struct.unpack('>I', data[0:4])[0]
        timestamp = struct.unpack('>f', data[4:8])[0]
        
        # Unpack joint angles
        joints = np.array([
            struct.unpack('>f', data[8:12])[0],
            struct.unpack('>f', data[12:16])[0],
            struct.unpack('>f', data[16:20])[0]
        ])
        
        # Unpack velocities
        velocities = np.array([
            struct.unpack('>f', data[20:24])[0],
            struct.unpack('>f', data[24:28])[0],
            struct.unpack('>f', data[28:32])[0]
        ])
        
        confidence = struct.unpack('>f', data[32:36])[0]
        checksum = struct.unpack('>I', data[36:40])[0]
        
        # Verify checksum
        expected_checksum = sum(data[:36]) & 0xFFFFFFFF
        if checksum != expected_checksum:
            raise ValueError("Checksum mismatch")
        
        return TeleopPacket(
            sequence=sequence,
            timestamp=timestamp,
            operator_joints=joints,
            operator_velocities=velocities,
            confidence=confidence
        )

