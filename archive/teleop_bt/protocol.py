"""
Network Protocol for Telemanipulation

Defines the communication protocol between ESP32 operator tracker and Raspberry Pi controller.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import numpy as np
import struct
import time
from ..sensors import TeleopPacket


@dataclass
class NetworkStats:
    """Network communication statistics."""
    packets_received: int = 0
    packets_lost: int = 0
    last_packet_time: float = 0.0
    latency_ms: float = 0.0
    jitter_ms: float = 0.0
    
    def update_latency(self, send_time: float, receive_time: float):
        """Update latency measurement."""
        self.latency_ms = (receive_time - send_time) * 1000.0
    
    def update_jitter(self, new_latency_ms: float):
        """Update jitter measurement."""
        if self.latency_ms > 0:
            self.jitter_ms = abs(new_latency_ms - self.latency_ms)
        self.latency_ms = new_latency_ms


class TeleopProtocol:
    """Protocol handler for teleoperation packets."""
    
    PACKET_SIZE = 40  # bytes
    HEADER_SIZE = 8   # sequence (4) + timestamp (4)
    PAYLOAD_SIZE = 28 # joints (12) + velocities (12) + confidence (4)
    CHECKSUM_SIZE = 4 # checksum
    
    def __init__(self):
        self.stats = NetworkStats()
        self.expected_sequence = 0
        
    def pack_packet(self, packet: TeleopPacket) -> bytes:
        """Pack TeleopPacket to UDP bytes."""
        return packet.pack()
    
    def unpack_packet(self, data: bytes) -> Optional[TeleopPacket]:
        """Unpack UDP bytes to TeleopPacket."""
        try:
            if len(data) != self.PACKET_SIZE:
                raise ValueError(f"Invalid packet size: {len(data)} bytes")
            
            packet = TeleopPacket.unpack(data)
            
            # Update statistics
            self._update_stats(packet)
            
            return packet
            
        except Exception as e:
            print(f"Failed to unpack packet: {e}")
            self.stats.packets_lost += 1
            return None
    
    def _update_stats(self, packet: TeleopPacket):
        """Update network statistics."""
        current_time = time.time()
        
        # Check for lost packets
        if self.stats.packets_received > 0:
            expected_seq = self.expected_sequence
            if packet.sequence > expected_seq:
                self.stats.packets_lost += packet.sequence - expected_seq
        
        self.expected_sequence = packet.sequence + 1
        self.stats.packets_received += 1
        self.stats.last_packet_time = current_time
        
        # Estimate latency (simplified - in practice would use more sophisticated method)
        if self.stats.last_packet_time > 0:
            latency = current_time - packet.timestamp
            self.stats.update_latency(packet.timestamp, current_time)
    
    def is_connection_alive(self, timeout_seconds: float = 0.2) -> bool:
        """Check if connection is alive based on last packet time."""
        if self.stats.last_packet_time == 0:
            return False
        
        return (time.time() - self.stats.last_packet_time) < timeout_seconds
    
    def get_stats(self) -> NetworkStats:
        """Get current network statistics."""
        return self.stats
    
    def reset_stats(self):
        """Reset network statistics."""
        self.stats = NetworkStats()
        self.expected_sequence = 0


def create_test_packet(sequence: int = 0) -> TeleopPacket:
    """Create a test packet for protocol testing."""
    return TeleopPacket(
        sequence=sequence,
        timestamp=time.time(),
        operator_joints=np.array([0.1, 0.2, 0.3]),
        operator_velocities=np.array([0.05, 0.1, 0.15]),
        confidence=0.9
    )


def test_protocol():
    """Test the protocol packing/unpacking."""
    print("Testing TeleopProtocol...")
    
    protocol = TeleopProtocol()
    
    # Create test packet
    packet = create_test_packet(123)
    
    # Pack and unpack
    packed = protocol.pack_packet(packet)
    unpacked = protocol.unpack_packet(packed)
    
    if unpacked is None:
        print("FAILED: Could not unpack packet")
        return False
    
    # Verify data integrity
    assert packet.sequence == unpacked.sequence, "Sequence mismatch"
    assert np.allclose(packet.operator_joints, unpacked.operator_joints), "Joint angles mismatch"
    assert np.allclose(packet.operator_velocities, unpacked.operator_velocities), "Velocities mismatch"
    assert abs(packet.confidence - unpacked.confidence) < 1e-6, "Confidence mismatch"
    
    print("PASSED: Protocol test successful")
    print(f"  Packed size: {len(packed)} bytes")
    print(f"  Stats: {protocol.get_stats()}")
    
    return True


if __name__ == "__main__":
    test_protocol()
