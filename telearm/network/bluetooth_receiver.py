"""
Bluetooth Serial Receiver for Telemanipulation

Non-blocking Bluetooth serial receiver for operator data from ESP32.
Mirrors the UDP receiver interface for compatibility.
"""
from __future__ import annotations
import serial
import threading
import time
from typing import Optional, Callable
from collections import deque
from .protocol import TeleopProtocol, NetworkStats
from ..sensors import TeleopPacket


class BluetoothOperatorDataReceiver:
    """Non-blocking Bluetooth serial receiver for operator data."""
    
    def __init__(self, port: str = "/dev/rfcomm0", baud: int = 115200, buffer_size: int = 10, timeout_seconds: float = 0.2):
        """
        Initialize Bluetooth serial receiver.
        
        Args:
            port: Bluetooth serial port (e.g., '/dev/rfcomm0')
            baud: Baud rate for serial communication
            buffer_size: Maximum number of packets to buffer
            timeout_seconds: Connection timeout for watchdog
        """
        self.port = port
        self.baud = baud
        self.buffer_size = buffer_size
        self.timeout_seconds = timeout_seconds
        
        # Serial communication
        self.serial = None
        
        # Network components
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
        """Start the Bluetooth serial receiver in background thread."""
        if self.running:
            return
        
        try:
            # Create serial connection
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,  # 100ms timeout for non-blocking operation
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Start receive thread
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            print(f"Bluetooth serial receiver started on {self.port} at {self.baud} baud")
            
        except Exception as e:
            print(f"Failed to start Bluetooth serial receiver: {e}")
            self.running = False
    
    def stop(self):
        """Stop the Bluetooth serial receiver."""
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
        
        print("Bluetooth serial receiver stopped")
    
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
        print("Bluetooth serial receive loop started")
        
        while self.running:
            try:
                # Read exactly 40 bytes (packet size)
                data = self.serial.read(40)
                
                if len(data) == 40:
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
                        print(f"Received packet {packet.sequence}, "
                              f"latency: {stats.latency_ms:.1f}ms, "
                              f"lost: {stats.packets_lost}")
                
                elif len(data) > 0:
                    # Partial packet - this shouldn't happen with fixed 40-byte packets
                    print(f"Warning: Received partial packet of {len(data)} bytes")
                
            except serial.SerialException as e:
                # Handle disconnection
                if self.is_connected:
                    self.is_connected = False
                    if self.timeout_callback:
                        try:
                            self.timeout_callback()
                        except Exception as e:
                            print(f"Timeout callback error: {e}")
                    print("Bluetooth connection lost")
                
                # Try to reconnect
                try:
                    if self.serial and not self.serial.is_open:
                        self.serial.open()
                        print("Bluetooth serial reconnected")
                except Exception as reconnect_error:
                    print(f"Failed to reconnect Bluetooth: {reconnect_error}")
                
                time.sleep(0.1)  # Brief delay before retry
                continue
                
            except Exception as e:
                if self.running:  # Only print errors if we're supposed to be running
                    print(f"Bluetooth receive error: {e}")
                time.sleep(0.1)
                continue
        
        print("Bluetooth serial receive loop stopped")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


def test_bluetooth_receiver():
    """Test the Bluetooth receiver with mock data."""
    print("Testing Bluetooth receiver...")
    
    # Note: This test requires actual Bluetooth hardware
    # For unit testing, use MockOperatorDataReceiver instead
    try:
        with BluetoothOperatorDataReceiver() as receiver:
            time.sleep(0.1)  # Let it try to connect
            
            packet = receiver.get_latest()
            if packet:
                print(f"Received packet: seq={packet.sequence}, "
                      f"joints={packet.operator_joints}, "
                      f"velocities={packet.operator_velocities}")
            else:
                print("No packets received (expected if no Bluetooth device)")
            
            stats = receiver.get_stats()
            print(f"Stats: {stats}")
    
    except Exception as e:
        print(f"Bluetooth test error (expected if no device): {e}")
    
    print("Bluetooth receiver test completed")


if __name__ == "__main__":
    test_bluetooth_receiver()
