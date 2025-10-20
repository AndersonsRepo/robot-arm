# Telearm Program Overview

This document provides a comprehensive overview of the Telearm teleoperation system architecture, file interactions, and troubleshooting guide for operators working with the physical system.

## System Architecture Overview

### High-Level Architecture
```
┌─────────────────┐    WiFi UDP /     ┌─────────────────┐    USB Serial    ┌─────────────────┐
│   ESP32 + IMUs  │ ── Bluetooth ──→ │  Raspberry Pi   │ ──────────────→ │    Arduino      │
│  (Operator)     │    SPP            │  (Controller)   │                 │   (Servos)      │
│                 │   100 Hz          │                 │                 │                 │
└─────────────────┘                   └─────────────────┘                 └─────────────────┘
```

### Communication Modes
- **WiFi Mode**: ESP32 → WiFi UDP → Raspberry Pi → USB Serial → Arduino
- **Bluetooth Mode**: ESP32 → Bluetooth SPP → Raspberry Pi → USB Serial → Arduino

## File Structure and Interactions

### Core Python Package (`telearm/`)

#### Main Control Flow
```
telearm/cli.py
    ↓
telearm/teleoperation/controller.py
    ↓
telearm/network/receiver.py (factory)
    ↓
telearm/network/bluetooth_receiver.py OR telearm/network/operator_data_receiver.py
    ↓
telearm/network/protocol.py
    ↓
telearm/sensors.py (TeleopPacket)
    ↓
telearm/imu_fusion.py (for mock data/testing)
```

#### Key Components

**1. CLI Interface (`telearm/cli.py`)**
- Entry point for all commands
- Parses command-line arguments
- Creates appropriate controllers
- Handles mode selection (WiFi/Bluetooth)

**2. Teleoperation Controller (`telearm/teleoperation/controller.py`)**
- Main control loop (100 Hz)
- Coordinates all subsystems
- Manages safety and emergency stops
- Integrates network, mapping, and hardware

**3. Network Layer (`telearm/network/`)**
- `receiver.py`: Factory for creating WiFi/Bluetooth receivers
- `bluetooth_receiver.py`: Bluetooth serial communication
- `operator_data_receiver.py`: WiFi UDP communication
- `protocol.py`: Packet serialization/deserialization

**4. Teleoperation Components (`telearm/teleoperation/`)**
- `mapper.py`: Maps operator velocities to robot velocities
- `integrator.py`: Integrates velocities to positions
- `controller.py`: Main teleoperation controller

**5. Safety System (`telearm/safety/`)**
- `checker.py`: Comprehensive safety validation
- Joint limits, workspace boundaries, emergency conditions

**6. Hardware Drivers (`telearm/drivers/`)**
- `serial_arduino.py`: Arduino servo control
- `null_driver.py`: Simulation mode

**7. IMU Fusion (`telearm/imu_fusion.py`)**
- `ComplementaryFilter`: Quaternion-based orientation estimation
- `OperatorPoseEstimator`: Multi-IMU pose estimation
- Mock data generation for testing and development
- Note: Production IMU fusion runs on ESP32 firmware at 200Hz

### Configuration Files (`config/`)

**1. Robot Configuration (`config/robot.yaml`)**
```yaml
robot:
  name: "telearm-5dof"
  dof: 5
joints:
  - name: "base_yaw"
    home: 0.0
    limit_min: -180.0
    limit_max: 180.0
```

**2. Hardware Configuration (`config/pins.yaml`)**
```yaml
hardware:
  arduino:
    baud_rate: 115200
    default_port: "/dev/ttyUSB0"
servos:
  pins: [3, 5, 6, 9, 10]
```

**3. Teleoperation Configuration (`config/teleop.yaml`)**
```yaml
teleoperation:
  mode: "wifi"  # or "bluetooth"
  velocity_scale: 0.6
  update_rate_hz: 100
  timeout_ms: 200
```

### Firmware (`firmware/`)

**1. ESP32 Operator Tracker (`firmware/esp32/operator_tracker/operator_tracker.ino`)**
- Reads 3× MPU-9250 IMU sensors
- Complementary filter for orientation estimation
- Transmits operator pose via WiFi UDP or Bluetooth SPP
- Mode selection via compile-time flags

**2. Arduino Servo Driver (`firmware/arduino/telearm_driver/telearm_driver.ino`)**
- Controls 5× servos via PWM
- Emergency stop and watchdog functionality
- Serial communication with Raspberry Pi

### Documentation (`docs/`)

- `setup.md`: Installation and setup guide
- `wiring.md`: Hardware wiring instructions
- `safety.md`: Safety procedures and guidelines
- `bluetooth-setup.md`: Bluetooth configuration and troubleshooting
- `program-overview.md`: This file

### Scripts (`scripts/`)

- `setup_bluetooth_pi.sh`: Automated Raspberry Pi Bluetooth pairing
- `telearm-bluetooth.service`: Systemd service for Bluetooth binding

## Data Flow and Packet Structure

### Packet Format (40 bytes)
```
| seq (4) | timestamp (4) | joints (12) | velocities (12) | confidence (4) | checksum (4) |
```

### Data Flow Process
1. **ESP32**: Reads IMUs → Complementary filter → Joint angles → UDP/Bluetooth packet
2. **Raspberry Pi**: Receives packet → Protocol unpacking → Velocity mapping → Safety checks → Arduino commands
3. **Arduino**: Receives commands → PWM servo control → Emergency monitoring

## Operational Procedures

### System Startup Sequence

#### 1. Hardware Power-Up
```bash
# Power on in this order:
# 1. Arduino (robot servos)
# 2. Raspberry Pi
# 3. ESP32 (operator tracker)
```

#### 2. Software Initialization
```bash
# On Raspberry Pi:
cd /path/to/robot-arm

# Check system status
python3 -m telearm.cli --sim status

# Test hardware connection
python3 -m telearm.cli status
```

#### 3. Communication Setup

**For WiFi Mode:**
```bash
# Ensure WiFi network is available
# ESP32 should connect automatically
# Check ESP32 Serial Monitor for connection status

# Start teleoperation
python3 -m telearm.cli teleop --mode wifi
```

**For Bluetooth Mode:**
```bash
# Pair ESP32 with Raspberry Pi
./scripts/setup_bluetooth_pi.sh

# Start teleoperation
python3 -m telearm.cli teleop --mode bluetooth
```

### Normal Operation

#### Starting Teleoperation
```bash
# WiFi mode (default)
python3 -m telearm.cli teleop

# Bluetooth mode
python3 -m telearm.cli teleop --mode bluetooth

# Mock mode (testing without hardware)
python3 -m telearm.cli teleop --mock
```

#### Monitoring System Status
The teleoperation controller provides real-time status:
```
Status: packets=1250, processed=1248, connection=OK
```

#### Emergency Procedures
- **Hardware E-stop**: Press emergency stop button (Arduino pin 13)
- **Software E-stop**: Press Ctrl+C in terminal
- **Network timeout**: Automatic emergency stop after 200ms timeout

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. ESP32 Connection Problems

**Issue**: ESP32 not connecting to WiFi/Bluetooth
**Symptoms**: 
- Serial Monitor shows "WiFi connection failed!" or "Bluetooth device ready for pairing"
- No packets received on Raspberry Pi

**Solutions**:
```bash
# Check ESP32 Serial Monitor (115200 baud)
# Verify network credentials in firmware
# Test network connectivity: ping <ESP32_IP>
# Check signal strength and range
```

#### 2. Raspberry Pi Communication Issues

**Issue**: No packets received from ESP32
**Symptoms**:
- "Connection timeout - no operator data received"
- Status shows connection=TIMEOUT

**Solutions**:
```bash
# Check network connectivity
ping <ESP32_IP>

# Monitor UDP traffic
sudo tcpdump -i wlan0 port 5000

# Check firewall settings
sudo ufw status

# Test with mock data
python3 -m telearm.cli teleop --mock
```

#### 3. Arduino Servo Control Problems

**Issue**: Servos not responding or erratic movement
**Symptoms**:
- Robot doesn't move with operator
- Jerky or delayed movement
- "Hardware communication error" messages

**Solutions**:
```bash
# Check Arduino connection
ls /dev/ttyUSB*

# Test Arduino communication
python3 -m telearm.cli status

# Check servo power supply (6A+ recommended)
# Verify servo wiring and PWM connections
```

#### 4. Safety System Issues

**Issue**: Emergency stops triggering unexpectedly
**Symptoms**:
- "Safety violation" messages
- Robot stops moving frequently
- Emergency stop warnings

**Solutions**:
```bash
# Check joint limits in config/robot.yaml
# Verify workspace boundaries
# Check for extreme operator movements
# Review safety configuration in config/teleop.yaml
```

### Diagnostic Commands

#### System Health Check
```bash
# Check all components
python3 -m telearm.cli status

# Test individual components
python3 -m telearm.cli --sim calibrate

# Check configuration loading
python3 -c "from telearm import load_from_config; print(load_from_config())"
```

#### Network Diagnostics
```bash
# WiFi diagnostics
iwconfig
ping <ESP32_IP>
sudo tcpdump -i wlan0 port 5000

# Bluetooth diagnostics
sudo bluetoothctl info <ESP32_MAC>
ls -la /dev/rfcomm*
```

#### Hardware Diagnostics
```bash
# Arduino connection
ls /dev/ttyUSB*
python3 -m telearm.cli status

# Servo power check
# Use multimeter to verify 5V supply
# Check servo current draw
```

### Performance Monitoring

#### Real-Time Metrics
The teleoperation system provides these metrics:
- **Packets received**: Network communication health
- **Packets processed**: Control loop performance
- **Safety violations**: Safety system activity
- **Connection status**: Communication reliability

#### Performance Targets
- **Control rate**: 100 Hz sustained
- **Latency**: <50ms end-to-end
- **Packet loss**: <2% acceptable
- **Safety violations**: Should be minimal

### Log Analysis

#### ESP32 Serial Monitor
Look for these key messages:
```
ESP32 Operator Tracker Starting...
Mode: WiFi UDP / Bluetooth Classic SPP
WiFi connected! / Bluetooth device ready for pairing
Sent packet X: angles=[...] vel=[...]
```

#### Python Console Output
Monitor for:
```
Starting teleoperation controller...
Control loop started at 100 Hz
Status: packets=X, processed=Y, connection=OK/TIMEOUT
Safety violation: [description]
```

### Configuration Management

#### Key Configuration Files
1. **`config/teleop.yaml`**: Communication mode, safety settings
2. **`config/robot.yaml`**: Robot parameters, joint limits
3. **`config/pins.yaml`**: Hardware configuration

#### Configuration Validation
```bash
# Test configuration loading
python3 -c "from telearm import load_from_config; print('Config OK')"

# Validate teleoperation config
python3 -c "import yaml; yaml.safe_load(open('config/teleop.yaml'))"
```

### Emergency Procedures

#### Immediate Safety Actions
1. **Hardware E-stop**: Press physical emergency stop button
2. **Software E-stop**: Ctrl+C in teleoperation terminal
3. **Power disconnect**: Remove power from servos
4. **Manual intervention**: Physically restrain robot if needed

#### Recovery Procedures
1. **Clear emergency conditions**: Check E-stop button, reset if needed
2. **Restart system**: Power cycle components in reverse order
3. **Verify connections**: Check all cables and connections
4. **Test components**: Use simulation mode first, then hardware

### Maintenance Procedures

#### Regular Maintenance
- **Weekly**: Check all connections and cables
- **Monthly**: Verify servo performance and calibration
- **Quarterly**: Update firmware and software
- **As needed**: Clean and lubricate mechanical components

#### Software Updates
```bash
# Update Python package
git pull
pip install -e ".[hardware]"

# Update ESP32 firmware
# Recompile and upload in Arduino IDE

# Update Arduino firmware
# Recompile and upload in Arduino IDE
```

## Advanced Configuration

### Custom Network Settings
Modify `firmware/esp32/operator_tracker/operator_tracker.ino`:
```cpp
const char* ssid = "YourNetwork";
const char* password = "YourPassword";
const char* targetIP = "192.168.1.100";
```

### Performance Tuning
Modify `config/teleop.yaml`:
```yaml
teleoperation:
  update_rate_hz: 100  # Control frequency
  timeout_ms: 200      # Network timeout
  velocity_scale: 0.6  # Robot speed scaling
```

### Safety Customization
```yaml
safety:
  enforce_joint_limits: true
  enforce_workspace: true
  soft_limit_margin_deg: 5.0
  emergency_stop_pin: 13
```

## Support and Resources

### Documentation References
- `docs/setup.md`: Installation guide
- `docs/wiring.md`: Hardware connections
- `docs/safety.md`: Safety procedures
- `docs/bluetooth-setup.md`: Bluetooth configuration
- `progress-doc-wifi/wifi-instructions.md`: WiFi setup

### Getting Help
1. Check this overview for common issues
2. Review relevant documentation files
3. Check ESP32 Serial Monitor for error messages
4. Test with mock data to isolate issues
5. Verify all connections and power supplies

### System Requirements
- **Raspberry Pi**: 4B with WiFi/Bluetooth
- **ESP32**: WROOM-32 with WiFi/Bluetooth
- **Arduino**: Uno R3
- **Power**: 6A+ supply for servos
- **Network**: WiFi router or direct Bluetooth pairing

This overview should provide operators with the knowledge needed to effectively operate and troubleshoot the Telearm teleoperation system in real-world conditions.
