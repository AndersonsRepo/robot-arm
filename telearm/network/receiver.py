"""
UDP Receiver for Telemanipulation

Non-blocking UDP receiver for operator data from ESP32.
"""
from __future__ import annotations
import socket
import threading
import time
from typing import Optional, Callable
from collections import deque
from .protocol import TeleopProtocol, NetworkStats
from ..sensors import TeleopPacket


class OperatorDataReceiver:
    """Non-blocking UDP receiver for operator data."""
    
    def __init__(self, port: int = 5000, buffer_size: int = 10, timeout_seconds: float = 0.2):
        """
        Initialize UDP receiver.
        
        Args:
            port: UDP port to listen on
            buffer_size: Maximum number of packets to buffer
            timeout_seconds: Connection timeout for watchdog
        """
        self.port = port
        self.buffer_size = buffer_size
        self.timeout_seconds = timeout_seconds
        
        # Network components
        self.socket = None
        self.protocol = TeleopProtocol()
        
        # Threading
        self.receive_thread = None
        self.running = False
        self.lock = threading.Lock()
        
        # Data buffer
        self.packet_buffer = deque(maxlen=buffer_size)
        
        # Callbacks
        self.packet_callback: Optional[Callable[[TeleopPacket], None]] = None
        self.timeout_callback: Optional[Callable[[], None]] = None
        
        # Statistics
        self.last_packet_time = 0.0
        self.is_connected = False
        
    def start(self):
        """Start the UDP receiver in background thread."""
        if self.running:
            return
        
        try:
            # Create UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.settimeout(0.1)  # 100ms timeout for non-blocking operation
            self.socket.bind(('', self.port))
            
            # Start receive thread
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            print(f"UDP receiver started on port {self.port}")
            
        except Exception as e:
            print(f"Failed to start UDP receiver: {e}")
            self.running = False
    
    def stop(self):
        """Stop the UDP receiver."""
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        if self.socket:
            self.socket.close()
            self.socket = None
        
        print("UDP receiver stopped")
    
    def get_latest(self) -> Optional[TeleopPacket]:
        """Get the most recent packet from buffer."""
        with self.lock:
            if self.packet_buffer:
                return self.packet_buffer[-1]
            return None
    
    def get_all_packets(self) -> list[TeleopPacket]:
        """Get all buffered packets and clear buffer."""
        with self.lock:
            packets = list(self.packet_buffer)
            self.packet_buffer.clear()
            return packets
    
    def is_connection_alive(self) -> bool:
        """Check if connection is alive based on recent packets."""
        return self.protocol.is_connection_alive(self.timeout_seconds)
    
    def get_stats(self) -> NetworkStats:
        """Get network statistics."""
        return self.protocol.get_stats()
    
    def set_packet_callback(self, callback: Callable[[TeleopPacket], None]):
        """Set callback for received packets."""
        self.packet_callback = callback
    
    def set_timeout_callback(self, callback: Callable[[], None]):
        """Set callback for connection timeout."""
        self.timeout_callback = callback
    
    def _receive_loop(self):
        """Main receive loop running in background thread."""
        print("UDP receive loop started")
        
        while self.running:
            try:
                # Receive packet
                data, addr = self.socket.recvfrom(1024)  # Buffer size for UDP
                
                # Unpack packet
                packet = self.protocol.unpack_packet(data)
                if packet is None:
                    continue
                
                # Update connection status
                self.last_packet_time = time.time()
                was_connected = self.is_connected
                self.is_connected = True
                
                # Add to buffer
                with self.lock:
                    self.packet_buffer.append(packet)
                
                # Call packet callback
                if self.packet_callback:
                    try:
                        self.packet_callback(packet)
                    except Exception as e:
                        print(f"Packet callback error: {e}")
                
                # Debug output
                if packet.sequence % 100 == 0:
                    stats = self.get_stats()
                    print(f"Received packet {packet.sequence} from {addr}, "
                          f"latency: {stats.latency_ms:.1f}ms, "
                          f"lost: {stats.packets_lost}")
                
            except socket.timeout:
                # Check for timeout
                if self.is_connected and not self.is_connection_alive():
                    self.is_connected = False
                    if self.timeout_callback:
                        try:
                            self.timeout_callback()
                        except Exception as e:
                            print(f"Timeout callback error: {e}")
                    print("Connection timeout - no packets received")
                
                continue
                
            except Exception as e:
                if self.running:  # Only print errors if we're supposed to be running
                    print(f"Receive error: {e}")
                continue
        
        print("UDP receive loop stopped")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


class MockOperatorDataReceiver:
    """Mock receiver for testing without hardware."""
    
    def __init__(self, port: int = 5000, buffer_size: int = 10):
        self.port = port
        self.buffer_size = buffer_size
        self.packet_buffer = deque(maxlen=buffer_size)
        self.running = False
        self.sequence = 0
        
        # Mock data generation
        self.mock_thread = None
        self.packet_callback: Optional[Callable[[TeleopPacket], None]] = None
        
    def start(self):
        """Start mock data generation."""
        if self.running:
            return
        
        self.running = True
        self.mock_thread = threading.Thread(target=self._mock_loop, daemon=True)
        self.mock_thread.start()
        
        print(f"Mock UDP receiver started on port {self.port}")
    
    def stop(self):
        """Stop mock data generation."""
        self.running = False
        if self.mock_thread and self.mock_thread.is_alive():
            self.mock_thread.join(timeout=1.0)
        print("Mock UDP receiver stopped")
    
    def get_latest(self) -> Optional[TeleopPacket]:
        """Get the most recent mock packet."""
        if self.packet_buffer:
            return self.packet_buffer[-1]
        return None
    
    def is_connection_alive(self) -> bool:
        """Always return True for mock."""
        return True
    
    def get_stats(self) -> NetworkStats:
        """Return mock stats."""
        return NetworkStats(
            packets_received=self.sequence,
            packets_lost=0,
            last_packet_time=time.time(),
            latency_ms=5.0,
            jitter_ms=1.0
        )
    
    def set_packet_callback(self, callback: Callable[[TeleopPacket], None]):
        """Set callback for mock packets."""
        self.packet_callback = callback
    
    def _mock_loop(self):
        """Generate mock packets."""
        from ..sensors import create_mock_operator_pose
        
        while self.running:
            try:
                # Create mock pose
                pose = create_mock_operator_pose()
                
                # Create packet
                packet = TeleopPacket(
                    sequence=self.sequence,
                    timestamp=time.time(),
                    operator_joints=pose.joint_angles,
                    operator_velocities=pose.joint_velocities,
                    confidence=pose.confidence
                )
                
                # Add to buffer
                self.packet_buffer.append(packet)
                
                # Call callback
                if self.packet_callback:
                    try:
                        self.packet_callback(packet)
                    except Exception as e:
                        print(f"Mock packet callback error: {e}")
                
                self.sequence += 1
                
                # 100 Hz = 10ms interval
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Mock loop error: {e}")
                time.sleep(0.1)
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


def test_receiver():
    """Test the UDP receiver with mock data."""
    print("Testing UDP receiver...")
    
    # Test with mock receiver
    with MockOperatorDataReceiver() as receiver:
        time.sleep(0.1)  # Let it generate some packets
        
        packet = receiver.get_latest()
        if packet:
            print(f"Received mock packet: seq={packet.sequence}, "
                  f"joints={packet.operator_joints}, "
                  f"velocities={packet.operator_velocities}")
        else:
            print("No packets received")
        
        stats = receiver.get_stats()
        print(f"Stats: {stats}")
    
    print("UDP receiver test completed")


if __name__ == "__main__":
    test_receiver()
