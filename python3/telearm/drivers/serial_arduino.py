"""
Serial Arduino Driver (v0)

Arduino serial driver for hobby servos.
"""
from __future__ import annotations
from typing import Optional
import numpy as np

# Optional: Serial-connected Arduino driver
try:
    import serial, time
except Exception:
    serial = None
    time = None

class SerialServoDriver:
    """Arduino serial driver. Sends ASCII 'M,<idx>,<angle_rad>
' and expects 'OK'.
    Use with the Arduino sketch further below.  Install with: pip install pyserial
    Args:
        n: number of joints
        port: serial device (e.g., '/dev/ttyUSB0', '/dev/tty.usbmodem14101', 'COM3')
        baud: must match the Arduino sketch
        timeout: read timeout seconds for 'OK' lines
        echo_ok: whether to read/ignore an 'OK' line after each command
    """
    def __init__(self, n:int, port:str="/dev/ttyUSB0", baud:int=115200, timeout:float=0.25, echo_ok:bool=True):
        if serial is None:
            raise ImportError("pyserial not installed; pip install pyserial")
        self._q = np.zeros(n)
        self._n = n
        self._echo_ok = echo_ok
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Arduino resets on open; small delay to allow bootloader
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def move_to(self, idx:int, angle_rad:float, speed:Optional[float]=None)->None:
        if idx < 0 or idx >= self._n:
            raise IndexError("joint index out of range")
        self._q[idx] = angle_rad
        cmd = f"M,{idx},{angle_rad:.6f}\n".encode()
        self.ser.write(cmd)
        if self._echo_ok:
            _ = self.ser.readline()  # consume 'OK' or error line

    def angles(self)->np.ndarray:
        # Until encoders/IMU are integrated, we return the last commanded values
        return self._q.copy()
