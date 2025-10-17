# Setup Guide

This guide covers the complete setup process for the Telearm robot system.

## Prerequisites

- Python 3.8 or higher
- Arduino IDE or compatible programming environment
- USB cable for Arduino programming
- External 5V power supply (see [Wiring Guide](wiring.md))

## Software Installation

### 1. Install Telearm Package

```bash
# Clone the repository
git clone <repository-url>
cd robot-arm

# Install in development mode
make install
# or
pip install -e .

# Install with hardware support
pip install -e ".[hardware]"

# Install development dependencies
make dev
```

### 2. Verify Installation

```bash
# Test the package
python -c "import telearm; print(telearm.__version__)"

# Run smoke test
python -m examples.smoke_test

# Test CLI
telearm --help
```

## Arduino Firmware Setup

### 1. Install Arduino IDE

Download from [arduino.cc](https://www.arduino.cc/en/software)

### 2. Upload Firmware

1. Connect Arduino to computer via USB
2. Open Arduino IDE
3. Open `firmware/arduino/telearm_driver/telearm_driver.ino`
4. Select correct board (Arduino Uno) and port
5. Click Upload

### 3. Verify Firmware

Open Serial Monitor (Tools → Serial Monitor):
- Set baud rate to 115200
- You should see "READY" message
- Test with: `M,0,0.0` (should return "OK")

## Configuration

### 1. Robot Parameters

Edit `config/robot.yaml` to match your hardware:

```yaml
joints:
  - name: "base_yaw"
    home: 0.0
    limit_min: -180.0  # Adjust based on physical limits
    limit_max: 180.0
```

### 2. Hardware Pins

Edit `config/pins.yaml` if using different Arduino pins:

```yaml
servos:
  pins: [3, 5, 6, 9, 10]  # Change if needed
```

## Hardware Assembly

### 1. Mechanical Assembly

Follow the mechanical assembly instructions for your specific robot kit:
- Mount servos according to joint specifications
- Ensure proper alignment and secure mounting
- Check for smooth movement without binding

### 2. Electrical Connections

Follow the [Wiring Guide](wiring.md):
- Connect servos to specified Arduino pins
- Use external 5V power supply
- Verify all connections are secure

### 3. Power On

1. Connect external power supply
2. Connect Arduino USB for communication
3. Verify servos move to center positions

## Calibration

### 1. Basic Calibration

```bash
# Run calibration routine
telearm calibrate

# Check status
telearm status
```

### 2. Joint Limit Calibration

1. Manually move each joint to its limits
2. Note the angles at physical limits
3. Update `config/robot.yaml` with actual limits
4. Test with CLI:

```bash
# Test joint limits
telearm joints 0 0 0 0 0  # Center
telearm joints 1.57 0 0 0 0  # Near limit
```

### 3. DH Parameter Calibration

If needed, measure actual link lengths and update `config/robot.yaml`:

```yaml
dh_parameters:
  links:
    - {a: 0.05, alpha: 90.0, d: 0.10}  # Measured values
```

## Testing

### 1. Basic Functionality

```bash
# Test in simulation mode
telearm --sim home
telearm --sim status
telearm --sim calibrate

# Test with hardware
telearm home
telearm status
```

### 2. Movement Testing

```bash
# Small movements
telearm move 0.05 0.0 0.04
telearm move 0.0 0.05 0.04

# Joint space movements
telearm joints 0.1 0.0 0.0 0.0 0.0
```

### 3. Full System Test

```bash
# Run comprehensive tests
make test

# Test examples
make examples
```

## ROS 2 Integration (Optional)

### 1. Install ROS 2

Follow ROS 2 installation guide for your system.

### 2. Build ROS 2 Package

```bash
# Ensure telearm is installed first
pip install -e .

# Build ROS 2 package
cd ros2/telearm_ros2
colcon build --packages-select telearm_ros2
```

### 3. Launch Visualization

```bash
# Source the workspace
source install/setup.bash

# Launch RViz
ros2 launch telearm_ros2 telearm_rviz.launch.py
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure telearm is installed with `pip install -e .`
2. **Serial Connection**: Check port and baud rate (115200)
3. **Servo Issues**: Verify power supply and wiring
4. **ROS 2 Issues**: Ensure telearm package is installed before building ROS 2 package

### Debug Commands

```bash
# Check Python installation
python -c "import telearm; print(telearm.__version__)"

# Test serial communication
python -c "import serial; print('Serial available')"

# Check config files
python -c "from telearm import load_robot_config; print(load_robot_config())"

# Test CLI without hardware
telearm --sim status
```

### Getting Help

1. Check the [Safety Guide](safety.md) for safety considerations
2. Review [Wiring Guide](wiring.md) for electrical issues
3. Run `make test` to verify software functionality
4. Use `telearm --help` for CLI options

## Next Steps

- Review [Safety Guide](safety.md) before operation
- Practice with small movements first
- Experiment with different trajectories
- Consider adding sensors for feedback control
