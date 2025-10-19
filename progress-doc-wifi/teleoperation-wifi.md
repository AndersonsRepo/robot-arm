# Teleoperation System Structure & Inner Workings

## System Architecture Overview

The teleoperation system implements a distributed architecture with three main components, supporting both WiFi and Bluetooth communication modes:

```
┌─────────────────┐    WiFi UDP /     ┌─────────────────┐    Serial     ┌─────────────────┐
│   ESP32 + IMUs  │ ── Bluetooth ──→ │  Raspberry Pi   │ ──────────── │    Arduino      │
│  (Operator)     │    SPP            │  (Controller)   │              │   (Servos)      │
│                 │   100 Hz          │                 │              │                 │
└─────────────────┘                   └─────────────────┘              └─────────────────┘
```

### Communication Modes
- **WiFi Mode**: ESP32 → WiFi UDP → Raspberry Pi → USB Serial → Arduino
- **Bluetooth Mode**: ESP32 → Bluetooth SPP → Raspberry Pi → USB Serial → Arduino

## Core Components & Data Flow

### 1. Operator Tracking (ESP32)
**Location**: `firmware/esp32/operator_tracker/operator_tracker.ino`

**Responsibilities**:
- Read 3× MPU-9250 IMU sensors via I2C
- Apply complementary filter for orientation estimation
- Estimate joint angles from IMU orientations
- Pack data into 40-byte packets
- Transmit at 100 Hz via WiFi UDP or Bluetooth SPP

**Data Flow**:
```
IMU Raw Data → Complementary Filter → Joint Angles → Packet → WiFi/Bluetooth
```

**Key Classes/Functions**:
- `FilterState` struct: Maintains quaternion state for each IMU
- `updateComplementaryFilter()`: Fuses accelerometer and gyroscope data
- `transmitOperatorPose()`: Creates and sends packets via WiFi or Bluetooth

### 2. Network Communication (Python)
**Location**: `telearm/network/`

#### Protocol Layer (`protocol.py`)
**Purpose**: Defines packet format and serialization

**Key Components**:
- `TeleopPacket` dataclass: 40-byte packet structure
- `pack()` method: Serializes to bytes
- `unpack()` method: Deserializes from bytes
- `NetworkStats` class: Tracks latency, jitter, packet loss

#### Receiver Factory (`receiver.py`)
**Purpose**: Creates appropriate receiver based on configuration

**Key Components**:
- `create_receiver()` function: Factory for WiFi/Bluetooth receivers
- `OperatorDataReceiver`: WiFi UDP receiver
- `BluetoothOperatorDataReceiver`: Bluetooth serial receiver

#### WiFi Receiver (`operator_data_receiver.py`)
**Purpose**: Receives operator data via WiFi UDP

#### Bluetooth Receiver (`bluetooth_receiver.py`)
**Purpose**: Receives operator data via Bluetooth serial

**Packet Format** (40 bytes):
```
| seq (4) | timestamp (4) | joints (12) | velocities (12) | confidence (4) | checksum (4) |
```

#### Receiver Layer (`receiver.py`)
**Purpose**: Non-blocking UDP reception with watchdog

**Key Components**:
- `OperatorDataReceiver`: Real UDP receiver
- `MockOperatorDataReceiver`: Testing without hardware
- Background thread for non-blocking reception
- Connection monitoring and timeout handling

**Data Flow**:
```
UDP Socket → Protocol Unpacking → Packet Buffer → Control Loop
```

### 3. Control Mapping (Python)
**Location**: `telearm/teleoperation/`

#### Velocity Mapper (`mapper.py`)
**Purpose**: Maps operator velocities to robot velocities using Jacobian-based control

**Key Algorithm**:
```python
def map_velocity(self, op_q, op_dq, robot_q):
    # 1. Compute operator end-effector velocity
    J_op = self.op_kin.jacobian_geometric(op_q)
    v_op_ee = J_op @ op_dq  # 6D twist
    
    # 2. Scale velocity
    v_robot_ee = self.velocity_scale * v_op_ee
    
    # 3. Solve robot joint velocities
    J_robot = self.robot_kin.jacobian_geometric(robot_q)
    robot_dq = self._solve_ik_velocity(J_robot, v_robot_ee, robot_q)
```

**Key Features**:
- Damped least squares for singularity handling
- Null-space bias toward home position
- Configurable velocity scaling (default 60%)

#### Velocity Integrator (`integrator.py`)
**Purpose**: Integrates joint velocities to positions with safety constraints

**Key Features**:
- Velocity/acceleration limiting
- Soft joint limits (gradual velocity reduction)
- Hard joint limits (position clamping)
- Smooth integration with configurable time step

### 4. Safety System (Python)
**Location**: `telearm/safety/`

#### Safety Checker (`checker.py`)
**Purpose**: Comprehensive safety constraint enforcement

**Safety Layers**:
1. **Joint Limits**: Soft and hard position limits
2. **Velocity Limits**: Maximum joint velocities
3. **Acceleration Limits**: Maximum joint accelerations
4. **Workspace Boundaries**: Cartesian space constraints
5. **Emergency Conditions**: Extreme value detection

**Key Functions**:
- `comprehensive_safety_check()`: Main safety validation
- `check_joint_limits()`: Position and velocity limiting
- `check_workspace()`: Cartesian boundary enforcement
- `check_emergency_conditions()`: Critical safety violations

### 5. Main Controller (Python)
**Location**: `telearm/teleoperation/controller.py`

**Purpose**: Orchestrates all components in real-time control loop

**Control Loop** (100 Hz):
```python
def _control_loop(self):
    while self.is_running:
        # 1. Get operator data
        packet = self.receiver.get_latest()
        
        # 2. Check emergency conditions
        if self.emergency_handler.is_emergency_active():
            self._handle_emergency()
            continue
            
        # 3. Map velocity
        robot_dq = self.mapper.map_velocity(...)
        
        # 4. Apply safety checks
        is_safe, robot_dq_safe, status = self.safety.comprehensive_safety_check(...)
        
        # 5. Integrate to positions
        self.robot_q, self.robot_dq = self.integrator.integrate(robot_dq_safe)
        
        # 6. Send to hardware
        self._send_to_hardware()
```

**State Management**:
- Robot joint positions and velocities
- Emergency stop status
- Network connection status
- Performance statistics

### 6. Hardware Interface (Arduino)
**Location**: `firmware/arduino/telearm_driver/telearm_driver.ino`

**Purpose**: Servo control with safety features

**Key Features**:
- 5× servo control via PWM
- Emergency stop on pin 13
- Watchdog timer (200ms timeout)
- Automatic servo disable on safety violations

**Safety Integration**:
- Hardware E-stop button
- Software watchdog from Pi
- Immediate servo disable on violations

## Data Models & Configuration

### Operator Arm Model
**Location**: `config/operator_arm.yaml`

**Structure**:
```yaml
operator_arm:
  name: "human-arm-3dof"
  dof: 3

segments:
  - name: "upper_arm"
    length: 0.30
    imu_id: 0
  # ... forearm, hand

dh_parameters:
  convention: "standard"
  links:
    - {a: 0.30, alpha: 0.0, d: 0.0}  # upper arm
    # ... other links
```

### Teleoperation Configuration
**Location**: `config/teleop.yaml`

**Key Parameters**:
- `velocity_scale`: Robot speed relative to operator (0.6 = 60%)
- `update_rate_hz`: Control loop frequency (100 Hz)
- `timeout_ms`: Network timeout (200ms)
- `null_space_gain`: Home bias strength (0.1)
- Safety settings: Joint limits, workspace, E-stop pin

### IMU Data Structures
**Location**: `telearm/sensors.py`

**Key Classes**:
- `IMUReading`: Raw sensor data (accel, gyro, mag, timestamp)
- `OperatorPose`: Fused pose data (joint angles, velocities, confidence)
- `TeleopPacket`: Network packet format

## Real-Time Performance

### Timing Requirements
- **ESP32 Fusion**: 200 Hz (5ms period)
- **ESP32 Transmission**: 100 Hz (10ms period)
- **Pi Control Loop**: 100 Hz (10ms period)
- **Arduino Servo Update**: 200 Hz (5ms period)
- **Total Latency Target**: <50ms end-to-end

### Performance Monitoring
- Network statistics (latency, jitter, packet loss)
- Control loop timing (lag detection)
- Safety violation tracking
- Connection status monitoring

## Error Handling & Recovery

### Network Errors
- Connection timeout → Emergency stop
- Packet corruption → Checksum validation
- High latency → Warning messages

### Safety Violations
- Joint limit exceeded → Velocity reduction/clipping
- Workspace violation → Motion restriction
- Emergency stop → Immediate servo disable

### Hardware Failures
- Arduino communication loss → Watchdog timeout
- Servo failure → Error reporting
- E-stop activation → System shutdown

## Integration Points

### CLI Integration
**Location**: `telearm/cli.py`

**Commands**:
- `telearm teleop`: Start teleoperation
- `telearm teleop --mock`: Test with simulated data
- `telearm teleop --config`: Custom configuration

### Testing Framework
**Location**: `tests/test_teleoperation.py`

**Test Coverage**:
- IMU fusion algorithms
- Network protocol
- Velocity mapping
- Safety checking
- End-to-end integration

### Configuration Management
- YAML-based configuration files
- Runtime parameter adjustment
- Environment-specific settings
- Validation and error checking

## Development & Debugging

### Mock Mode
- Synthetic operator data generation
- No hardware dependencies
- Performance testing
- Algorithm validation

### Debug Output
- Real-time status messages
- Performance statistics
- Safety violation reports
- Network health monitoring

### Logging & Monitoring
- Packet sequence tracking
- Latency measurement
- Error rate monitoring
- System health indicators

This architecture provides a robust, real-time teleoperation system with comprehensive safety features and flexible configuration options.
