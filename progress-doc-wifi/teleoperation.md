# Telemanipulation System Documentation

## Overview

The Telearm telemanipulation system enables real-time control of a 5-DOF robot arm using IMU-tracked operator arm movements. The system implements velocity-based Jacobian mapping with comprehensive safety constraints and real-time communication.

## System Architecture

```
┌─────────────────┐    Wi-Fi UDP     ┌─────────────────┐    Serial     ┌─────────────────┐
│   ESP32 + IMUs  │ ──────────────── │  Raspberry Pi   │ ──────────── │    Arduino      │
│  (Operator)     │   100 Hz         │  (Controller)   │              │   (Servos)      │
└─────────────────┘                  └─────────────────┘              └─────────────────┘
```

### Components

1. **ESP32 Operator Tracker**: Reads 3× MPU-9250 IMUs, fuses data, transmits pose via UDP
2. **Raspberry Pi Controller**: Receives operator data, maps to robot velocities, enforces safety
3. **Arduino Servo Driver**: Controls 5 servos with watchdog and emergency stop

## Hardware Setup

### ESP32 Operator Tracker

**Hardware Requirements:**
- ESP32-WROOM-32 development board
- 3× MPU-9250 IMU sensors
- Breadboard and jumper wires
- 3.3V power supply

**Pinout:**
```
ESP32 Pin    →    MPU-9250 Pin
21 (SDA)     →    SDA
22 (SCL)     →    SCL
3.3V         →    VCC
GND          →    GND
```

**IMU Placement:**
- IMU 0 (0x68): Upper arm
- IMU 1 (0x69): Forearm  
- IMU 2 (0x68): Hand

**Network Configuration:**
- SSID: `TelearmNetwork`
- Password: `telearm123`
- Target IP: `192.168.1.100` (Raspberry Pi)
- UDP Port: `5000`

### Raspberry Pi Setup

**Hardware Requirements:**
- Raspberry Pi 4
- MicroSD card (32GB+)
- Power supply
- Wi-Fi connection

**Software Installation:**
```bash
# Install telearm with teleoperation support
pip install -e ".[teleop]"

# Install system dependencies
sudo apt update
sudo apt install python3-serial
```

### Arduino Servo Driver

**Hardware Requirements:**
- Arduino Uno/Nano
- 5× hobby servos (SG90 or similar)
- External power supply for servos
- Emergency stop button

**Pinout:**
```
Arduino Pin  →    Servo
3            →    Joint 0 (base_yaw)
5            →    Joint 1 (base_pitch)
6            →    Joint 2 (elbow)
9            →    Joint 3 (wrist_pitch)
10           →    Joint 4 (wrist_roll)
13           →    Emergency stop button
```

## Software Configuration

### Configuration Files

**`config/teleop.yaml`** - Teleoperation settings:
```yaml
teleoperation:
  velocity_scale: 0.6  # Robot moves at 60% operator speed
  update_rate_hz: 100  # Control loop frequency
  timeout_ms: 200      # Watchdog timeout
  
  mapping:
    type: "velocity_based"
    null_space_gain: 0.1  # Bias toward home in null space
    
  safety:
    enforce_joint_limits: true
    enforce_workspace: true
    soft_limit_margin_deg: 5.0
    emergency_stop_pin: 13  # Arduino pin for E-stop
```

**`config/operator_arm.yaml`** - Operator arm model:
```yaml
operator_arm:
  name: "human-arm-3dof"
  dof: 3
  
segments:
  - name: "upper_arm"
    length: 0.30  # meters
    imu_id: 0
  - name: "forearm"  
    length: 0.25
    imu_id: 1
  - name: "hand"
    length: 0.08
    imu_id: 2

dh_parameters:
  convention: "standard"
  links:
    - {a: 0.30, alpha: 0.0, d: 0.0}  # upper arm
    - {a: 0.25, alpha: 0.0, d: 0.0}  # forearm
    - {a: 0.08, alpha: 0.0, d: 0.0}  # hand
```

## Usage

### Starting Teleoperation

**With Hardware:**
```bash
telearm teleop --config config/teleop.yaml
```

**With Mock Data (Testing):**
```bash
telearm teleop --mock
```

### Calibration

**IMU Calibration:**
1. Place operator arm in known position
2. Run calibration routine
3. Verify joint angle estimates

**Network Latency Testing:**
```bash
# Test UDP communication
python -c "from telearm.network.receiver import test_receiver; test_receiver()"
```

## Safety Features

### Emergency Stop
- **Hardware**: Button on Arduino pin 13
- **Software**: Watchdog timeout (200ms)
- **Behavior**: Immediately disables all servos

### Joint Limits
- **Soft Limits**: Reduce velocity near boundaries
- **Hard Limits**: Stop motion at boundaries
- **Configuration**: Defined in `config/robot.yaml`

### Workspace Boundaries
- **Type**: Box-shaped workspace
- **Limits**: Defined in `config/robot.yaml`
- **Behavior**: Reduce velocity when outside workspace

### Velocity/Acceleration Limits
- **Per-joint limits**: Defined in robot model
- **Enforcement**: Applied in real-time
- **Safety margin**: 80% of maximum values

## Performance Specifications

### Timing Requirements
- **ESP32 Fusion**: 200 Hz (5ms period)
- **ESP32 Transmission**: 100 Hz (10ms period)  
- **Pi Control Loop**: 100 Hz (10ms period)
- **Arduino Servo Update**: 200 Hz (5ms period)
- **Total Latency**: <50ms end-to-end

### Network Protocol
- **Packet Size**: 40 bytes
- **Format**: Sequence, timestamp, joints (12B), velocities (12B), confidence (4B), checksum (4B)
- **Error Handling**: Checksum verification, sequence tracking

## Troubleshooting

### Common Issues

**1. No Operator Data Received**
```
Connection timeout - no operator data received
```
- Check ESP32 Wi-Fi connection
- Verify network configuration
- Test with mock data: `telearm teleop --mock`

**2. High Latency**
```
Warning: Control loop lagging by 15.2ms
```
- Reduce control loop frequency
- Check network congestion
- Optimize system resources

**3. Safety Violations**
```
Safety violation: Joint 2 velocity too high
```
- Check operator movement speed
- Adjust velocity scaling
- Verify joint limit configuration

**4. Arduino Communication Error**
```
Hardware communication error: Serial timeout
```
- Check USB connection
- Verify Arduino firmware
- Test with: `telearm status --sim`

### Debug Commands

**Test Network:**
```bash
python -c "from telearm.network.receiver import test_receiver; test_receiver()"
```

**Test IMU Fusion:**
```bash
python -c "from telearm.imu_fusion import test_imu_fusion; test_imu_fusion()"
```

**Test Velocity Mapping:**
```bash
python -c "from telearm.teleoperation.mapper import test_velocity_mapper; test_velocity_mapper()"
```

**Run Full Test Suite:**
```bash
pytest tests/test_teleoperation.py -v
```

## Development

### Adding New Features

**1. New Safety Constraints:**
- Extend `SafetyChecker` class
- Add configuration parameters
- Update tests

**2. Different Mapping Strategies:**
- Implement new mapper class
- Add configuration options
- Update controller integration

**3. Additional Sensors:**
- Extend `IMUReading` dataclass
- Update fusion algorithms
- Modify network protocol

### Code Structure

```
telearm/
├── sensors.py              # IMU data structures
├── imu_fusion.py          # Sensor fusion algorithms
├── network/
│   ├── protocol.py        # Network packet format
│   └── receiver.py        # UDP receiver
├── teleoperation/
│   ├── mapper.py          # Velocity mapping
│   ├── integrator.py      # Velocity integration
│   └── controller.py      # Main controller
├── safety/
│   └── checker.py         # Safety constraints
└── cli.py                 # Command-line interface
```

## Safety Guidelines

### Before Operation
1. **Verify Emergency Stop**: Test button functionality
2. **Check Joint Limits**: Ensure servos can move freely
3. **Clear Workspace**: Remove obstacles around robot
4. **Test Communication**: Verify all systems connected

### During Operation
1. **Monitor Status**: Watch for safety warnings
2. **Stay Alert**: Be ready to press emergency stop
3. **Smooth Movements**: Avoid sudden operator motions
4. **Regular Breaks**: Prevent operator fatigue

### After Operation
1. **Return to Home**: Move robot to safe position
2. **Power Down**: Disconnect power supplies
3. **Store Safely**: Secure all components

## Support

For technical support or bug reports:
1. Check this documentation first
2. Run diagnostic commands
3. Check system logs
4. Report issues with full error messages

## Future Enhancements

### Planned Features
- **Haptic Feedback**: Force feedback to operator
- **Multi-Operator**: Support multiple operators
- **Advanced Fusion**: Kalman filtering for IMUs
- **Visual Feedback**: Camera-based pose estimation
- **Machine Learning**: Adaptive mapping strategies

### Performance Improvements
- **Lower Latency**: <20ms target latency
- **Higher Frequency**: 200 Hz control loop
- **Better Accuracy**: Sub-degree joint precision
- **Robust Communication**: Redundant network paths
