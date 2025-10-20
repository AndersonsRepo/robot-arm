# Data Hierarchy Tree

This document shows the data flow and hierarchy within the Telearm teleoperation system.

## System Data Flow

```
┌─────────────────┐    Raw IMU Data     ┌─────────────────┐
│   MPU-9250      │ ──────────────────→ │   ESP32         │
│   Sensors       │    I2C (400kHz)     │   Operator      │
│   (3× units)    │                     │   Tracker       │
└─────────────────┘                     └─────────────────┘
                                                │
                                                │ Processed Data
                                                ▼
┌─────────────────┐    Operator Pose    ┌─────────────────┐
│   Raspberry Pi  │ ←─────────────────── │   ESP32         │
│   Controller    │   WiFi UDP /         │   (40-byte      │
│                 │   Bluetooth SPP      │    packets)     │
└─────────────────┘   (100 Hz)           └─────────────────┘
         │
         │ Robot Commands
         ▼
┌─────────────────┐    Servo Commands    ┌─────────────────┐
│   Arduino       │ ←─────────────────── │   Raspberry Pi  │
│   Servo Driver  │   USB Serial         │   Controller    │
│   (5× servos)   │   (115200 baud)      │                 │
└─────────────────┘                      └─────────────────┘
```

## Data Structure Hierarchy

### 1. Raw Sensor Data (ESP32)

```
IMU Reading
├── imu_id: int (0, 1, 2)
├── timestamp: float (seconds)
├── accel: float[3] (m/s²)
├── gyro: float[3] (rad/s)
├── mag: float[3] (μT)
└── temperature: float (°C)
```

### 2. Processed Orientation Data (ESP32)

```
Orientation (per IMU)
├── timestamp: float
├── quaternion: float[4] (w, x, y, z)
├── euler_angles: float[3] (roll, pitch, yaw)
├── imu_id: int
└── confidence: float (0-1)
```

### 3. Operator Pose Data (ESP32 → Raspberry Pi)

```
OperatorPose
├── sequence: uint32_t (packet counter)
├── timestamp: float (seconds)
├── joint_angles: float[3] (radians)
│   ├── shoulder_pitch
│   ├── elbow_angle
│   └── wrist_angle
├── joint_velocities: float[3] (rad/s)
│   ├── shoulder_velocity
│   ├── elbow_velocity
│   └── wrist_velocity
└── confidence: float (0-1)
```

### 4. Network Packet Format (40 bytes)

```
TeleopPacket (Binary Format)
├── sequence: uint32_t (4 bytes)
├── timestamp: float (4 bytes)
├── joint_angles: float[3] (12 bytes)
├── joint_velocities: float[3] (12 bytes)
├── confidence: float (4 bytes)
└── checksum: uint32_t (4 bytes)
```

### 5. Robot Configuration Data

```
RobotModel
├── name: str
├── dof: int (5)
├── joints: JointSpec[5]
│   ├── name: str
│   ├── home: float (radians)
│   ├── limit_min: float
│   ├── limit_max: float
│   ├── max_velocity: float
│   └── max_acceleration: float
└── dh_parameters: DH[5]
    ├── a: float (link length)
    ├── alpha: float (link twist)
    ├── d: float (link offset)
    └── theta: float (joint angle)
```

### 6. Operator Model Data

```
OperatorModel
├── name: str ("human_arm")
├── dof: int (3)
├── joints: JointSpec[3]
│   ├── shoulder_pitch
│   ├── elbow_angle
│   └── wrist_angle
└── imu_placements: IMUPlacement[3]
    ├── imu_id: int
    ├── body_segment: str
    └── mounting_orientation: float[3]
```

### 7. Teleoperation Configuration

```
TeleopConfig
├── mode: str ("wifi" | "bluetooth")
├── velocity_scale: float (0.6)
├── update_rate_hz: int (100)
├── timeout_ms: int (200)
├── wifi_config: WiFiConfig
│   ├── port: int (5000)
│   └── target_ip: str
├── bluetooth_config: BluetoothConfig
│   ├── port: str ("/dev/rfcomm0")
│   └── baud_rate: int (115200)
├── mapping: MappingConfig
│   ├── type: str ("velocity_based")
│   └── null_space_gain: float (0.1)
└── safety: SafetyConfig
    ├── enforce_joint_limits: bool
    ├── enforce_workspace: bool
    ├── soft_limit_margin_deg: float (5.0)
    └── emergency_stop_pin: int (13)
```

### 8. Robot State Data

```
RobotState
├── joint_positions: float[5] (radians)
├── joint_velocities: float[5] (rad/s)
├── joint_accelerations: float[5] (rad/s²)
├── end_effector_pose: Pose4D
│   ├── position: float[3] (x, y, z)
│   └── orientation: float[4] (quaternion)
├── jacobian: float[6×5] (6D velocity mapping)
└── timestamp: float
```

### 9. Safety System Data

```
SafetyStatus
├── joint_limits_ok: bool
├── workspace_ok: bool
├── emergency_stop_active: bool
├── watchdog_alive: bool
├── violation_count: int
├── last_violation: str
└── timestamp: float
```

### 10. Network Statistics

```
NetworkStats
├── packets_received: int
├── packets_lost: int
├── latency_ms: float
├── jitter_ms: float
├── throughput_bps: float
├── connection_uptime: float
└── last_packet_time: float
```

## Data Processing Pipeline

### ESP32 Processing Chain

```
Raw IMU Data → Complementary Filter → Joint Angles → Packet Creation → Transmission
     ↓                ↓                    ↓              ↓              ↓
  I2C Reading    Orientation Est.    Kinematics      Serialization   WiFi/BT
  (200 Hz)       (200 Hz)           (100 Hz)        (100 Hz)        (100 Hz)
```

### Raspberry Pi Processing Chain

```
Network Packet → Protocol Unpack → Velocity Mapping → Safety Check → Servo Commands
      ↓              ↓                ↓                ↓              ↓
   Reception      Deserialization   Jacobian         Validation    Serial TX
   (100 Hz)       (100 Hz)         (100 Hz)         (100 Hz)      (100 Hz)
```

## Data Storage Hierarchy

### Configuration Files (YAML)

```
config/
├── robot.yaml          # Robot model parameters
├── pins.yaml           # Hardware pin assignments
├── teleop.yaml         # Teleoperation settings
└── operator_arm.yaml   # Operator model parameters
```

### Runtime Data

```
Memory Buffers
├── IMU Buffer (ESP32)
│   ├── Raw readings: 200 Hz × 3 IMUs
│   └── Orientation: 200 Hz × 3 IMUs
├── Packet Buffer (Raspberry Pi)
│   ├── Operator packets: 10 packets max
│   └── Robot commands: 10 commands max
└── State Buffer (Raspberry Pi)
    ├── Robot state: 100 Hz
    └── Safety status: 100 Hz
```

### Log Data

```
Logs/
├── teleoperation.log   # Main operation log
├── network.log        # Network statistics
├── safety.log         # Safety violations
└── debug.log          # Debug information
```

## Data Validation Hierarchy

### Input Validation

```
Raw Data Validation
├── IMU Range Checks
│   ├── Accelerometer: ±16g
│   ├── Gyroscope: ±2000 dps
│   └── Magnetometer: ±4800 μT
├── Timestamp Validation
│   ├── Monotonic increase
│   └── Reasonable delta (1-20ms)
└── Checksum Validation
    └── CRC32 verification
```

### Processing Validation

```
Processed Data Validation
├── Quaternion Normalization
├── Joint Angle Limits
├── Velocity Bounds
├── Confidence Range (0-1)
└── Sequence Continuity
```

### Output Validation

```
Command Validation
├── Servo Angle Limits
├── Velocity Constraints
├── Acceleration Limits
├── Emergency Stop Check
└── Watchdog Timeout
```

## Performance Metrics Hierarchy

### Latency Metrics

```
End-to-End Latency
├── IMU Reading: 5ms
├── ESP32 Processing: 2ms
├── Network Transmission: 5-20ms
│   ├── WiFi UDP: 5-15ms
│   └── Bluetooth SPP: 10-20ms
├── Raspberry Pi Processing: 3ms
└── Arduino Execution: 1ms
Total: 16-31ms (target: <50ms)
```

### Throughput Metrics

```
Data Rates
├── IMU Raw Data: 2.4 KB/s (3×200Hz×4bytes)
├── Processed Packets: 4 KB/s (100Hz×40bytes)
├── Network Bandwidth: 32 Kbps (100Hz×40bytes×8bits)
└── Servo Commands: 0.5 KB/s (100Hz×5bytes)
```

This data hierarchy provides a complete view of how data flows through the Telearm system, from raw sensor readings to final servo commands.
