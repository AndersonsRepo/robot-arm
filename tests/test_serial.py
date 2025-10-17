"""Tests for serial driver (mocked when pyserial not available)."""
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock

# Try to import pyserial, skip tests if not available
try:
    import serial
    PYSERIAL_AVAILABLE = True
except ImportError:
    PYSERIAL_AVAILABLE = False


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_creation():
    """Test SerialServoDriver creation with mocked serial."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        # Mock the serial connection
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        driver = SerialServoDriver(n=5, port="/dev/ttyUSB0")
        
        assert driver._n == 5
        assert driver.ser == mock_serial_instance
        mock_serial.assert_called_once()


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_move_to():
    """Test SerialServoDriver move_to method."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        driver = SerialServoDriver(n=3, port="/dev/ttyUSB0")
        
        # Test move_to
        driver.move_to(0, np.pi/4)
        
        # Should have written command
        mock_serial_instance.write.assert_called()
        written_data = mock_serial_instance.write.call_args[0][0]
        
        # Should be properly formatted
        command = written_data.decode()
        assert command.startswith("M,0,")
        assert "0.785398" in command  # pi/4 approximation
        
        # Should have read response if echo_ok is True
        mock_serial_instance.readline.assert_called()


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_angles():
    """Test SerialServoDriver angles method."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        driver = SerialServoDriver(n=3, port="/dev/ttyUSB0")
        
        # Move some joints
        driver.move_to(0, 0.5)
        driver.move_to(1, 1.0)
        driver.move_to(2, 1.5)
        
        # Get angles
        angles = driver.angles()
        
        assert len(angles) == 3
        assert angles[0] == 0.5
        assert angles[1] == 1.0
        assert angles[2] == 1.5


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_bounds_checking():
    """Test SerialServoDriver bounds checking."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        driver = SerialServoDriver(n=3, port="/dev/ttyUSB0")
        
        # Test valid indices
        driver.move_to(0, 0.0)  # Should work
        driver.move_to(2, 0.0)  # Should work
        
        # Test invalid indices
        with pytest.raises(IndexError):
            driver.move_to(-1, 0.0)  # Negative index
        
        with pytest.raises(IndexError):
            driver.move_to(3, 0.0)   # Index too large


def test_serial_driver_import_error():
    """Test SerialServoDriver when pyserial is not available."""
    if PYSERIAL_AVAILABLE:
        pytest.skip("pyserial is available")
    
    # This should raise ImportError when pyserial is not available
    with pytest.raises(ImportError):
        from telearm.drivers.serial_arduino import SerialServoDriver
        SerialServoDriver(n=3, port="/dev/ttyUSB0")


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_timeout_handling():
    """Test SerialServoDriver timeout handling."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        # Mock timeout scenario
        mock_serial_instance.readline.return_value = b''  # Empty response
        
        driver = SerialServoDriver(n=3, port="/dev/ttyUSB0", timeout=0.1)
        
        # Should not raise exception even with timeout
        driver.move_to(0, 0.0)


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_echo_ok_option():
    """Test SerialServoDriver echo_ok option."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        # Test with echo_ok=False
        driver_no_echo = SerialServoDriver(n=3, port="/dev/ttyUSB0", echo_ok=False)
        driver_no_echo.move_to(0, 0.0)
        
        # Should not call readline
        mock_serial_instance.readline.assert_not_called()
        
        # Reset mock
        mock_serial_instance.reset_mock()
        
        # Test with echo_ok=True (default)
        driver_with_echo = SerialServoDriver(n=3, port="/dev/ttyUSB0", echo_ok=True)
        driver_with_echo.move_to(0, 0.0)
        
        # Should call readline
        mock_serial_instance.readline.assert_called()


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_command_format():
    """Test SerialServoDriver command format."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial:
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        driver = SerialServoDriver(n=5, port="/dev/ttyUSB0", echo_ok=False)
        
        # Test command format
        driver.move_to(2, 1.234567)
        
        written_data = mock_serial_instance.write.call_args[0][0]
        command = written_data.decode()
        
        # Should be properly formatted: "M,<idx>,<angle>\n"
        assert command == "M,2,1.234567\n"


@pytest.mark.skipif(not PYSERIAL_AVAILABLE, reason="pyserial not available")
def test_serial_driver_initialization_delay():
    """Test SerialServoDriver initialization delay."""
    from telearm.drivers.serial_arduino import SerialServoDriver
    
    with patch('serial.Serial') as mock_serial, \
         patch('time.sleep') as mock_sleep:
        
        mock_serial_instance = MagicMock()
        mock_serial.return_value = mock_serial_instance
        
        SerialServoDriver(n=3, port="/dev/ttyUSB0")
        
        # Should have called sleep for Arduino bootloader delay
        mock_sleep.assert_called_with(2.0)
