"""
ESP32 Serial Driver for TeleArm

Binary packet communication with ESP32 motor driver firmware.
Replaces ASCII-based Arduino communication with compact binary protocol.
"""
from __future__ import annotations
import struct
import time
import threading
from typing import Optional, Callable
from enum import IntEnum

# Try to import pyserial
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    serial = None


class MessageType(IntEnum):
    """Message types for ESP32 protocol."""
    JOINT_TARGET = 0x01
    HOME = 0x02
    PARK = 0x03
    GRIPPER = 0x04
    STATUS_REQUEST = 0x05


class ESP32SerialDriver:
    """ESP32 serial driver with binary protocol."""
    
    # Protocol constants
    SYNC_BYTE = 0xAA
    PACKET_HEADER_SIZE = 2  # sync + type
    JOINT_PAYLOAD_SIZE = 10  # 5 joints * int16 (2 bytes each)
    GRIPPER_SIZE = 1
    CRC_SIZE = 2
    
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, 
                 timeout: float = 0.3, num_joints: int = 5):
        """
        Initialize ESP32 serial driver.
        
        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0", "COM3")
            baud: Baud rate (default 115200)
            timeout: Read/write timeout in seconds
            num_joints: Number of joints (default 5 for 5-DOF arm)
        """
        if not SERIAL_AVAILABLE:
            raise ImportError("pyserial not installed; pip install pyserial")
        
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.num_joints = num_joints
        
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.lock = threading.Lock()
        self.last_send_time = 0.0
        
        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.errors = 0
        
    def connect(self) -> bool:
        """Connect to ESP32 via serial."""
        if self.connected:
            return True
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                write_timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Wait for ESP32 to be ready (it may reset on connection)
            time.sleep(2.0)
            self.serial.reset_input_buffer()
            self.connected = True
            
            print(f"Connected to ESP32 on {self.port} at {self.baud} baud")
            return True
            
        except Exception as e:
            print(f"Failed to connect to ESP32: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from ESP32."""
        with self.lock:
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.connected = False
            print("Disconnected from ESP32")
    
    def send_joint_targets(self, joints_deg: list[float], gripper: Optional[int] = None) -> bool:
        """
        Send joint target angles to ESP32.
        
        Args:
            joints_deg: List of 5 joint angles in degrees
            gripper: Optional gripper value (0-255, None to omit)
            
        Returns:
            True if sent successfully
        """
        if not self.connected:
            if not self.connect():
                return False
        
        if len(joints_deg) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(joints_deg)}")
        
        # Build packet
        packet = bytearray()
        packet.append(self.SYNC_BYTE)
        packet.append(MessageType.JOINT_TARGET)
        
        # Pack joint angles as int16 (degrees * 100 for precision)
        for angle_deg in joints_deg:
            # Convert to centidegrees and clamp to int16 range
            centidegrees = int(round(angle_deg * 100.0))
            centidegrees = max(-32768, min(32767, centidegrees))
            packet.extend(struct.pack('>h', centidegrees))  # big-endian int16
        
        # Add gripper if specified
        if gripper is not None:
            packet.append(min(255, max(0, int(gripper))))
        else:
            # No gripper field - ESP32 should keep current gripper state
            pass
        
        # Calculate CRC16 (simple checksum for now)
        crc = self._calculate_crc16(packet)
        packet.extend(struct.pack('>H', crc))  # big-endian uint16
        
        # Send packet
        try:
            with self.lock:
                self.serial.write(packet)
                self.serial.flush()
                self.packets_sent += 1
                self.last_send_time = time.time()
                return True
        except Exception as e:
            print(f"Failed to send joint targets: {e}")
            self.errors += 1
            self.connected = False
            return False
    
    def send_home_command(self) -> bool:
        """Send home command to ESP32."""
        return self._send_command(MessageType.HOME)
    
    def send_park_command(self) -> bool:
        """Send park command to ESP32."""
        return self._send_command(MessageType.PARK)
    
    def send_gripper_command(self, value: int) -> bool:
        """
        Send gripper command.
        
        Args:
            value: Gripper value (0-255, 0=open, 255=closed)
        """
        if not self.connected:
            if not self.connect():
                return False
        
        packet = bytearray()
        packet.append(self.SYNC_BYTE)
        packet.append(MessageType.GRIPPER)
        packet.append(min(255, max(0, int(value))))
        
        crc = self._calculate_crc16(packet)
        packet.extend(struct.pack('>H', crc))
        
        try:
            with self.lock:
                self.serial.write(packet)
                self.serial.flush()
                self.packets_sent += 1
                return True
        except Exception as e:
            print(f"Failed to send gripper command: {e}")
            self.errors += 1
            self.connected = False
            return False
    
    def _send_command(self, msg_type: MessageType) -> bool:
        """Send a simple command packet."""
        if not self.connected:
            if not self.connect():
                return False
        
        packet = bytearray()
        packet.append(self.SYNC_BYTE)
        packet.append(msg_type)
        
        crc = self._calculate_crc16(packet)
        packet.extend(struct.pack('>H', crc))
        
        try:
            with self.lock:
                self.serial.write(packet)
                self.serial.flush()
                self.packets_sent += 1
                self.last_send_time = time.time()
                return True
        except Exception as e:
            print(f"Failed to send command {msg_type}: {e}")
            self.errors += 1
            self.connected = False
            return False
    
    def _calculate_crc16(self, data: bytearray) -> int:
        """Calculate simple CRC16 checksum."""
        # Simple CRC16-CCITT implementation
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc
    
    def read_status(self) -> Optional[dict]:
        """Read status message from ESP32 (if available)."""
        if not self.connected:
            return None
        
        try:
            with self.lock:
                if self.serial.in_waiting > 0:
                    # Read available data (ESP32 may send status messages)
                    data = self.serial.read(self.serial.in_waiting)
                    # For now, just return raw data
                    # TODO: Parse status messages if ESP32 sends them
                    return {'raw_data': data.hex()}
        except Exception as e:
            print(f"Error reading status: {e}")
        
        return None
    
    def is_connected(self) -> bool:
        """Check if connected to ESP32."""
        return self.connected and self.serial and self.serial.is_open
    
    def get_stats(self) -> dict:
        """Get communication statistics."""
        return {
            'packets_sent': self.packets_sent,
            'packets_received': self.packets_received,
            'errors': self.errors,
            'last_send_time': self.last_send_time,
            'connected': self.connected
        }
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


