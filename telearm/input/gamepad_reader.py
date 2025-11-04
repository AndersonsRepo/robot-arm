"""
USB Gamepad Reader for TeleArm

Reads USB gamepad input using evdev (primary) or pygame (fallback).
Provides normalized stick axes and button state polling.
"""
from __future__ import annotations
import time
import threading
from typing import Dict, Optional, Tuple
from dataclasses import dataclass, field

# Try evdev first (more efficient for Linux)
try:
    import evdev
    from evdev import InputDevice, categorize, ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False

# Fallback to pygame
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


@dataclass
class GamepadState:
    """Current gamepad state."""
    axes: Dict[int, float] = field(default_factory=dict)  # axis_index -> value (-1.0 to 1.0)
    buttons: Dict[int, bool] = field(default_factory=dict)  # button_index -> pressed
    timestamp: float = 0.0


class GamepadReader:
    """USB gamepad reader with evdev/pygame support."""
    
    def __init__(self, device_path: Optional[str] = None, deadzone: float = 0.1, use_evdev: bool = True):
        """
        Initialize gamepad reader.
        
        Args:
            device_path: Path to gamepad device (e.g., "/dev/input/js0")
                        If None or "auto", attempts auto-detection
            deadzone: Deadzone for stick axes (0.0-1.0)
            use_evdev: Prefer evdev if available (Linux), otherwise use pygame
        """
        self.deadzone = deadzone
        self.device_path = device_path
        self.use_evdev = use_evdev and EVDEV_AVAILABLE
        
        # State
        self.state = GamepadState()
        self.lock = threading.Lock()
        self.running = False
        self.read_thread: Optional[threading.Thread] = None
        
        # Backend
        self.device = None
        self.pygame_initialized = False
        
        # Auto-detect device if needed
        if device_path is None or device_path == "auto":
            self.device_path = self._auto_detect_device()
        
        if self.device_path:
            self._initialize()
    
    def _auto_detect_device(self) -> Optional[str]:
        """Auto-detect gamepad device."""
        if self.use_evdev:
            # Try common joystick devices
            for i in range(10):
                path = f"/dev/input/js{i}"
                try:
                    dev = InputDevice(path)
                    if 'js' in dev.name.lower() or 'gamepad' in dev.name.lower() or 'controller' in dev.name.lower():
                        print(f"Auto-detected gamepad: {dev.name} at {path}")
                        return path
                except (OSError, PermissionError):
                    continue
        
        # Try pygame if evdev failed
        if PYGAME_AVAILABLE:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                print(f"Auto-detected {pygame.joystick.get_count()} gamepad(s) via pygame")
                return "pygame"  # Special marker for pygame
        
        return None
    
    def _initialize(self):
        """Initialize the gamepad device."""
        if self.use_evdev and self.device_path and self.device_path != "pygame":
            try:
                self.device = InputDevice(self.device_path)
                print(f"Using evdev gamepad: {self.device.name}")
            except (OSError, PermissionError) as e:
                print(f"Failed to open {self.device_path}: {e}")
                self.device = None
                self.use_evdev = False
        
        if not self.use_evdev and PYGAME_AVAILABLE:
            if not self.pygame_initialized:
                pygame.init()
                pygame.joystick.init()
                self.pygame_initialized = True
            
            if pygame.joystick.get_count() > 0:
                self.device = pygame.joystick.Joystick(0)
                self.device.init()
                print(f"Using pygame gamepad: {self.device.get_name()}")
            else:
                raise RuntimeError("No gamepad detected")
    
    def start(self):
        """Start reading gamepad input in background thread."""
        if self.running:
            return
        
        if self.device is None:
            raise RuntimeError("Gamepad device not initialized")
        
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
    
    def stop(self):
        """Stop reading gamepad input."""
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.use_evdev and hasattr(self.device, 'close'):
            try:
                self.device.close()
            except:
                pass
    
    def get_state(self) -> GamepadState:
        """Get current gamepad state (thread-safe)."""
        with self.lock:
            return GamepadState(
                axes=self.state.axes.copy(),
                buttons=self.state.buttons.copy(),
                timestamp=self.state.timestamp
            )
    
    def get_axis(self, index: int, apply_deadzone: bool = True) -> float:
        """Get normalized axis value with deadzone applied."""
        with self.lock:
            value = self.state.axes.get(index, 0.0)
        
        if apply_deadzone and abs(value) < self.deadzone:
            return 0.0
        
        # Apply deadzone scaling (maps [deadzone, 1.0] -> [0.0, 1.0])
        if apply_deadzone and abs(value) > self.deadzone:
            sign = 1.0 if value >= 0 else -1.0
            abs_value = abs(value)
            scaled = (abs_value - self.deadzone) / (1.0 - self.deadzone)
            return sign * scaled
        
        return value
    
    def get_button(self, index: int) -> bool:
        """Get button state."""
        with self.lock:
            return self.state.buttons.get(index, False)
    
    def _read_loop(self):
        """Background thread for reading gamepad input."""
        if self.use_evdev:
            self._read_loop_evdev()
        else:
            self._read_loop_pygame()
    
    def _read_loop_evdev(self):
        """Read loop using evdev."""
        try:
            # Set device to non-blocking
            self.device.grab()  # Exclusive access
            
            for event in self.device.read_loop():
                if not self.running:
                    break
                
                with self.lock:
                    if event.type == ecodes.EV_ABS:
                        # Absolute axis (joystick)
                        axis_code = event.code
                        # Normalize to -1.0 to 1.0 (evdev values depend on axis)
                        if axis_code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY):
                            # Typical joystick range is -32768 to 32767
                            value = event.value / 32767.0
                            self.state.axes[axis_code] = max(-1.0, min(1.0, value))
                        elif axis_code in (ecodes.ABS_Z, ecodes.ABS_RZ):
                            # Triggers: 0 to 255 -> 0.0 to 1.0
                            value = event.value / 255.0
                            self.state.axes[axis_code] = max(0.0, min(1.0, value))
                        else:
                            self.state.axes[axis_code] = event.value
                    elif event.type == ecodes.EV_KEY:
                        # Button
                        self.state.buttons[event.code] = bool(event.value)
                    
                    self.state.timestamp = time.time()
        
        except Exception as e:
            print(f"Gamepad read error: {e}")
        finally:
            try:
                self.device.ungrab()
            except:
                pass
    
    def _read_loop_pygame(self):
        """Read loop using pygame."""
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                pygame.event.pump()
                
                with self.lock:
                    # Read axes (normalized to -1.0 to 1.0)
                    num_axes = self.device.get_numaxes()
                    for i in range(num_axes):
                        value = self.device.get_axis(i)
                        self.state.axes[i] = float(value)
                    
                    # Read buttons
                    num_buttons = self.device.get_numbuttons()
                    for i in range(num_buttons):
                        self.state.buttons[i] = bool(self.device.get_button(i))
                    
                    self.state.timestamp = time.time()
                
                clock.tick(100)  # 100 Hz polling
        
        except Exception as e:
            print(f"Gamepad read error: {e}")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()

