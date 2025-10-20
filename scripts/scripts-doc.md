# Telearm Scripts Documentation

This document provides comprehensive information about all scripts in the Telearm robot arm project, their purposes, usage, and interactions.

## Overview

The Telearm project includes several scripts to automate setup, deployment, and operation of the robot arm system. These scripts handle everything from Bluetooth pairing to system startup and maintenance.

## Scripts Directory Structure

```
scripts/
├── start_telearm.sh           # Main startup script
├── setup_bluetooth_pi.sh     # Bluetooth pairing automation
├── telearm-bluetooth.service  # Systemd service for Bluetooth
└── scripts-doc.md            # This documentation file
```

## Script Details

### 1. `start_telearm.sh` - Main Startup Script

**Purpose**: Unified startup script for the Telearm robot system with automatic mode detection and configuration validation.

**Features**:
- Automatic dependency checking and installation
- Configuration validation
- Multiple operation modes (WiFi, Bluetooth, Simulation)
- Optional ROS2 visualization
- Verbose logging
- Signal handling and cleanup

**Usage**:
```bash
# Basic usage
./scripts/start_telearm.sh [mode] [options]

# Examples
./scripts/start_telearm.sh wifi --rviz
./scripts/start_telearm.sh bluetooth --verbose
./scripts/start_telearm.sh sim
```

**Modes**:
- `wifi` - WiFi teleoperation (default)
- `bluetooth` - Bluetooth teleoperation
- `sim` - Simulation mode (no hardware)

**Options**:
- `--rviz` - Enable ROS2 visualization
- `--config <file>` - Use custom config file
- `--verbose` - Enable verbose output
- `--help` - Show help message

**Dependencies**:
- Python 3.8+
- telearm package
- pyserial (for hardware modes)
- bluetoothctl (for Bluetooth mode)
- ros2 (for visualization)

**Interactions**:
- Calls `setup_bluetooth_pi.sh` if Bluetooth setup needed
- Validates all configuration files
- Starts ROS2 visualization if requested
- Launches teleoperation via `telearm.cli`

### 2. `setup_bluetooth_pi.sh` - Bluetooth Pairing Script

**Purpose**: Automates the Bluetooth pairing process between Raspberry Pi and ESP32 for teleoperation communication.

**Features**:
- Automatic device scanning
- MAC address detection
- Pairing and trust establishment
- Serial port binding (`/dev/rfcomm0`)
- Permission setting
- Connection testing

**Usage**:
```bash
# With automatic scanning
sudo ./scripts/setup_bluetooth_pi.sh

# With specific MAC address
sudo ./scripts/setup_bluetooth_pi.sh AA:BB:CC:DD:EE:FF
```

**Dependencies**:
- bluetoothctl (BlueZ package)
- rfcomm (BlueZ package)
- Root privileges

**Output**:
- Creates `/dev/rfcomm0` serial port
- Sets permissions (666)
- Provides connection test results

**Interactions**:
- Called by `start_telearm.sh` when Bluetooth mode is selected
- Used by `telearm-bluetooth.service` for persistent connections

### 3. `telearm-bluetooth.service` - Systemd Service

**Purpose**: Systemd service for automatic Bluetooth RFCOMM binding on system startup.

**Features**:
- Automatic startup after Bluetooth service
- Persistent Bluetooth connection
- Clean shutdown handling
- Service management integration

**Usage**:
```bash
# Edit service file with ESP32 MAC address
sudo nano scripts/telearm-bluetooth.service

# Install and enable service
sudo cp scripts/telearm-bluetooth.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable telearm-bluetooth.service
sudo systemctl start telearm-bluetooth.service
```

**Configuration**:
- Replace `<ESP32_MAC_ADDRESS>` with actual ESP32 MAC
- Uses fixed RFCOMM channel 1 for compatibility
- 10-second delay to allow Bluetooth initialization

**Interactions**:
- Provides persistent Bluetooth connection for `start_telearm.sh`
- Automatically binds ESP32 to `/dev/rfcomm0` on boot

## Script Interactions and Workflow

### Development Workflow

1. **Local Development**:
   ```bash
   # Install dependencies
   make dev
   
   # Test in simulation
   ./scripts/start_telearm.sh sim --verbose
   ```

2. **Hardware Testing**:
   ```bash
   # Test with hardware (WiFi mode)
   ./scripts/start_telearm.sh wifi --rviz
   
   # Test with hardware (Bluetooth mode)
   ./scripts/start_telearm.sh bluetooth --verbose
   ```

### Production Deployment Workflow

1. **Initial Setup**:
   ```bash
   # Install telearm package
   make install
   
   # Setup Bluetooth (if using Bluetooth mode)
   sudo ./scripts/setup_bluetooth_pi.sh
   
   # Enable persistent Bluetooth service
   sudo systemctl enable telearm-bluetooth.service
   ```

2. **Daily Operation**:
   ```bash
   # Start robot system
   ./scripts/start_telearm.sh wifi
   
   # Or with visualization
   ./scripts/start_telearm.sh bluetooth --rviz
   ```

### Error Handling and Troubleshooting

#### Common Issues

1. **Bluetooth Connection Failed**:
   ```bash
   # Check Bluetooth status
   sudo bluetoothctl show
   
   # Re-run setup
   sudo ./scripts/setup_bluetooth_pi.sh
   ```

2. **Arduino Not Detected**:
   ```bash
   # Check USB devices
   ls -la /dev/ttyUSB* /dev/ttyACM*
   
   # Check permissions
   ls -la /dev/ttyUSB0
   ```

3. **ROS2 Visualization Issues**:
   ```bash
   # Check ROS2 installation
   ros2 --version
   
   # Test launch file
   ros2 launch telearm_ros2 telearm_rviz.launch.py
   ```

#### Debug Mode

Use verbose mode for detailed logging:
```bash
./scripts/start_telearm.sh wifi --verbose
```

## Configuration Files

The scripts interact with several configuration files:

### Required Configuration Files

1. **`config/robot.yaml`**:
   - Robot model definition
   - Joint specifications
   - DH parameters
   - Workspace limits

2. **`config/pins.yaml`**:
   - Arduino pin assignments
   - Servo configurations
   - Emergency stop settings

3. **`config/teleop.yaml`**:
   - Teleoperation parameters
   - Velocity mapping settings
   - Safety configurations

### Optional Configuration Files

1. **`config/operator_arm.yaml`**:
   - Operator arm model
   - IMU sensor configurations
   - Calibration data

## Security Considerations

### Permissions

- Bluetooth setup requires root privileges
- Serial port access needs proper permissions
- Service files require system-level installation

### Network Security

- WiFi mode uses UDP communication
- Bluetooth uses SPP (Serial Port Profile)
- No encryption implemented (consider for production)

## Performance Considerations

### Resource Usage

- **Python telearm**: ~50MB RAM, minimal CPU
- **ROS2 visualization**: ~200MB RAM, moderate CPU
- **Bluetooth service**: ~10MB RAM, minimal CPU

### Optimization Tips

1. **Disable visualization** for production:
   ```bash
   ./scripts/start_telearm.sh wifi  # No --rviz flag
   ```

2. **Use simulation mode** for testing:
   ```bash
   ./scripts/start_telearm.sh sim
   ```

3. **Monitor system resources**:
   ```bash
   htop
   ```

## Maintenance and Updates

### Regular Maintenance

1. **Check service status**:
   ```bash
   sudo systemctl status telearm-bluetooth.service
   ```

2. **Update dependencies**:
   ```bash
   pip3 install --upgrade telearm
   ```

3. **Clean logs**:
   ```bash
   sudo journalctl --vacuum-time=7d
   ```

### Updates

When updating the system:

1. **Stop services**:
   ```bash
   sudo systemctl stop telearm-bluetooth.service
   ```

2. **Update code**:
   ```bash
   git pull
   make install
   ```

3. **Restart services**:
   ```bash
   sudo systemctl start telearm-bluetooth.service
   ```

## Troubleshooting Guide

### Script Execution Issues

1. **Permission Denied**:
   ```bash
   chmod +x scripts/start_telearm.sh
   ```

2. **Command Not Found**:
   ```bash
   # Check PATH
   echo $PATH
   
   # Use full path
   /path/to/scripts/start_telearm.sh
   ```

3. **Python Import Errors**:
   ```bash
   # Reinstall package
   pip3 install -e .
   
   # Check Python path
   python3 -c "import sys; print(sys.path)"
   ```

### Hardware Issues

1. **Arduino Connection**:
   - Check USB cable
   - Verify Arduino power
   - Check serial port permissions

2. **ESP32 Communication**:
   - Verify WiFi/Bluetooth connectivity
   - Check IP address configuration
   - Monitor network traffic

3. **Servo Control**:
   - Check power supply
   - Verify pin connections
   - Test individual servos

## Best Practices

### Development

1. **Always test in simulation first**
2. **Use verbose mode for debugging**
3. **Keep configuration files backed up**
4. **Document any custom modifications**

### Production

1. **Use systemd services for persistence**
2. **Monitor system resources**
3. **Implement proper logging**
4. **Have emergency stop procedures**

### Security

1. **Limit network access**
2. **Use proper file permissions**
3. **Regular security updates**
4. **Monitor system logs**

## Support and Contributing

### Getting Help

1. **Check logs**:
   ```bash
   journalctl -u telearm-bluetooth.service
   ```

2. **Run diagnostics**:
   ```bash
   ./scripts/start_telearm.sh sim --verbose
   ```

3. **Test components individually**:
   ```bash
   python3 -m telearm.cli status
   ```

### Contributing

When adding new scripts:

1. **Follow naming conventions** (`script_name.sh`)
2. **Include proper error handling**
3. **Add usage documentation**
4. **Update this documentation file**
5. **Test on both development and production systems**

---

*This documentation is maintained alongside the Telearm project. Please update it when adding or modifying scripts.*
