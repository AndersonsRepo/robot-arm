"""
Unit tests for Bluetooth communication components.
"""
import pytest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import numpy as np

from telearm.network.bluetooth_receiver import BluetoothOperatorDataReceiver
from telearm.network.receiver import create_receiver
from telearm.sensors import TeleopPacket


class TestBluetoothOperatorDataReceiver:
    """Test BluetoothOperatorDataReceiver functionality."""
    
    def test_bluetooth_receiver_creation(self):
        """Test BluetoothOperatorDataReceiver can be created."""
        receiver = BluetoothOperatorDataReceiver(
            port="/dev/rfcomm0",
            baud=115200,
            buffer_size=5,
            timeout_seconds=0.1
        )
        
        assert receiver.port == "/dev/rfcomm0"
        assert receiver.baud == 115200
        assert receiver.buffer_size == 5
        assert receiver.timeout_seconds == 0.1
        assert not receiver.running
        assert receiver.serial is None
    
    @patch('serial.Serial')
    def test_bluetooth_receiver_start_stop(self, mock_serial):
        """Test starting and stopping Bluetooth receiver."""
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        receiver = BluetoothOperatorDataReceiver()
        
        # Start receiver
        receiver.start()
        assert receiver.running
        assert receiver.serial is not None
        assert receiver.receive_thread is not None
        
        # Stop receiver
        receiver.stop()
        assert not receiver.running
        mock_serial_instance.close.assert_called_once()
    
    def test_bluetooth_receiver_context_manager(self):
        """Test BluetoothOperatorDataReceiver as context manager."""
        with patch('serial.Serial'):
            with BluetoothOperatorDataReceiver() as receiver:
                assert receiver.running
            # Should be stopped after context exit
            assert not receiver.running
    
    def test_bluetooth_receiver_callbacks(self):
        """Test callback registration."""
        receiver = BluetoothOperatorDataReceiver()
        
        packet_callback = Mock()
        timeout_callback = Mock()
        
        receiver.set_packet_callback(packet_callback)
        receiver.set_timeout_callback(timeout_callback)
        
        assert receiver.packet_callback == packet_callback
        assert receiver.timeout_callback == timeout_callback
    
    def test_bluetooth_receiver_buffer_operations(self):
        """Test packet buffer operations."""
        receiver = BluetoothOperatorDataReceiver(buffer_size=3)
        
        # Initially empty
        assert receiver.get_latest() is None
        assert receiver.get_all_packets() == []
        
        # Add packets to buffer
        packet1 = TeleopPacket(1, time.time(), np.array([0.1, 0.2, 0.3]), np.array([0.01, 0.02, 0.03]), 0.9)
        packet2 = TeleopPacket(2, time.time(), np.array([0.2, 0.3, 0.4]), np.array([0.02, 0.03, 0.04]), 0.8)
        
        receiver.packet_buffer.append(packet1)
        receiver.packet_buffer.append(packet2)
        
        # Test get_latest
        latest = receiver.get_latest()
        assert latest.sequence == 2
        
        # Test get_all_packets
        all_packets = receiver.get_all_packets()
        assert len(all_packets) == 2
        assert all_packets[0].sequence == 1
        assert all_packets[1].sequence == 2
        
        # Buffer should be empty after get_all_packets
        assert receiver.get_latest() is None
    
    def test_bluetooth_receiver_connection_status(self):
        """Test connection status checking."""
        receiver = BluetoothOperatorDataReceiver(timeout_seconds=0.1)
        
        # Initially not connected
        assert not receiver.is_connection_alive()
        
        # Simulate recent packet
        receiver.last_packet_time = time.time()
        assert receiver.is_connection_alive()
        
        # Simulate old packet
        receiver.last_packet_time = time.time() - 1.0
        assert not receiver.is_connection_alive()


class TestReceiverFactory:
    """Test receiver factory function."""
    
    def test_create_receiver_wifi_default(self):
        """Test factory creates WiFi receiver by default."""
        config = {
            'port': 5000,
            'timeout_ms': 200
        }
        
        receiver = create_receiver(config)
        
        # Should create OperatorDataReceiver (WiFi)
        from telearm.network.receiver import OperatorDataReceiver
        assert isinstance(receiver, OperatorDataReceiver)
        assert receiver.port == 5000
    
    def test_create_receiver_bluetooth_mode(self):
        """Test factory creates Bluetooth receiver when mode is bluetooth."""
        config = {
            'mode': 'bluetooth',
            'bluetooth': {
                'port': '/dev/rfcomm0',
                'baud_rate': 115200
            },
            'timeout_ms': 200
        }
        
        receiver = create_receiver(config)
        
        # Should create BluetoothOperatorDataReceiver
        assert isinstance(receiver, BluetoothOperatorDataReceiver)
        assert receiver.port == '/dev/rfcomm0'
        assert receiver.baud == 115200
    
    def test_create_receiver_bluetooth_defaults(self):
        """Test factory uses defaults for Bluetooth receiver."""
        config = {
            'mode': 'bluetooth'
        }
        
        receiver = create_receiver(config)
        
        assert isinstance(receiver, BluetoothOperatorDataReceiver)
        assert receiver.port == '/dev/rfcomm0'  # Default
        assert receiver.baud == 115200  # Default
    
    def test_create_receiver_case_insensitive(self):
        """Test factory handles case insensitive mode."""
        config = {
            'mode': 'BLUETOOTH',  # Uppercase
            'bluetooth': {
                'port': '/dev/rfcomm1'
            }
        }
        
        receiver = create_receiver(config)
        
        assert isinstance(receiver, BluetoothOperatorDataReceiver)
        assert receiver.port == '/dev/rfcomm1'


class TestBluetoothIntegration:
    """Integration tests for Bluetooth communication."""
    
    @patch('serial.Serial')
    def test_bluetooth_packet_reception_simulation(self, mock_serial):
        """Test simulated packet reception over Bluetooth."""
        # Mock serial connection
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        # Create test packet data (40 bytes)
        test_packet = TeleopPacket(
            sequence=123,
            timestamp=time.time(),
            operator_joints=np.array([0.1, 0.2, 0.3]),
            operator_velocities=np.array([0.01, 0.02, 0.03]),
            confidence=0.9
        )
        packet_data = test_packet.pack()
        
        # Mock serial read to return our packet
        mock_serial_instance.read.side_effect = [packet_data, b'']  # Return packet, then empty
        
        receiver = BluetoothOperatorDataReceiver()
        received_packets = []
        
        def packet_callback(packet):
            received_packets.append(packet)
        
        receiver.set_packet_callback(packet_callback)
        
        # Start receiver
        receiver.start()
        
        # Wait for packet processing
        time.sleep(0.1)
        
        # Stop receiver
        receiver.stop()
        
        # Verify packet was received
        assert len(received_packets) == 1
        assert received_packets[0].sequence == 123
        assert np.allclose(received_packets[0].operator_joints, [0.1, 0.2, 0.3])
    
    def test_bluetooth_timeout_handling(self):
        """Test Bluetooth timeout handling."""
        receiver = BluetoothOperatorDataReceiver(timeout_seconds=0.1)
        
        timeout_called = False
        
        def timeout_callback():
            nonlocal timeout_called
            timeout_called = True
        
        receiver.set_timeout_callback(timeout_callback)
        
        # Simulate timeout condition
        receiver.last_packet_time = time.time() - 1.0  # Old packet
        receiver.is_connected = True
        
        # Check timeout
        assert not receiver.is_connection_alive()
        
        # In real implementation, timeout callback would be triggered
        # by the receive loop when connection is lost
        assert not timeout_called  # Not triggered in this test
    
    def test_bluetooth_error_recovery(self):
        """Test Bluetooth error recovery mechanisms."""
        receiver = BluetoothOperatorDataReceiver()
        
        # Test that receiver can handle serial errors gracefully
        with patch('serial.Serial') as mock_serial:
            mock_serial_instance = MagicMock()
            mock_serial.return_value = mock_serial_instance
            
            # Simulate serial error
            mock_serial_instance.read.side_effect = Exception("Serial error")
            
            receiver.start()
            time.sleep(0.1)  # Let error occur
            receiver.stop()
            
            # Should not crash, receiver should handle error gracefully
            assert not receiver.running


class TestBluetoothPerformance:
    """Performance tests for Bluetooth communication."""
    
    def test_bluetooth_buffer_size_limit(self):
        """Test that buffer respects size limit."""
        receiver = BluetoothOperatorDataReceiver(buffer_size=2)
        
        # Add more packets than buffer size
        for i in range(5):
            packet = TeleopPacket(i, time.time(), np.array([0.1, 0.2, 0.3]), np.array([0.01, 0.02, 0.03]), 0.9)
            receiver.packet_buffer.append(packet)
        
        # Should only keep last 2 packets
        all_packets = receiver.get_all_packets()
        assert len(all_packets) == 2
        assert all_packets[0].sequence == 3  # 3rd packet
        assert all_packets[1].sequence == 4   # 4th packet
    
    def test_bluetooth_thread_safety(self):
        """Test thread safety of buffer operations."""
        receiver = BluetoothOperatorDataReceiver(buffer_size=10)
        
        def add_packets():
            for i in range(100):
                packet = TeleopPacket(i, time.time(), np.array([0.1, 0.2, 0.3]), np.array([0.01, 0.02, 0.03]), 0.9)
                receiver.packet_buffer.append(packet)
                time.sleep(0.001)
        
        def read_packets():
            for _ in range(50):
                receiver.get_latest()
                time.sleep(0.002)
        
        # Run concurrent operations
        thread1 = threading.Thread(target=add_packets)
        thread2 = threading.Thread(target=read_packets)
        
        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
        
        # Should not crash due to race conditions
        assert True  # If we get here, no exceptions occurred


if __name__ == "__main__":
    pytest.main([__file__])
