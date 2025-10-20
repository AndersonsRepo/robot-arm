# File Hierarchy Tree

This document shows the complete file structure and organization of the Telearm robot arm control system.

## Project Root Structure

```
robot-arm/
├── README.md                           # Main project documentation
├── pyproject.toml                      # Python package configuration
├── requirements.txt                    # Python dependencies
├── Makefile                           # Build and development commands
├── pull-request-message.md            # PR template for GitHub
│
├── config/                            # Configuration files
│   ├── robot.yaml                     # Robot model parameters
│   ├── pins.yaml                      # Hardware pin assignments
│   ├── teleop.yaml                    # Teleoperation settings
│   └── operator_arm.yaml              # Operator model parameters
│
├── docs/                              # Documentation
│   ├── setup.md                       # Complete setup guide
│   ├── wiring.md                      # Hardware wiring instructions
│   ├── safety.md                      # Safety procedures and guidelines
│   ├── bluetooth-setup.md             # Bluetooth configuration guide
│   ├── program-overview.md            # System architecture overview
│   ├── data-hierarchy.md              # Data flow and structure
│   └── file-hierarchy.md              # This file
│
├── firmware/                          # Microcontroller firmware
│   ├── arduino/                       # Arduino Uno firmware
│   │   └── telearm_driver/
│   │       └── telearm_driver.ino     # Servo control firmware
│   └── esp32/                         # ESP32 operator tracker firmware
│       └── operator_tracker/
│           └── operator_tracker.ino  # IMU fusion and communication
│
├── telearm/                           # Main Python package
│   ├── __init__.py                    # Package initialization
│   ├── cli.py                         # Command-line interface
│   ├── control.py                     # Motion control algorithms
│   ├── ik.py                          # Inverse kinematics solver
│   ├── imu_fusion.py                  # IMU fusion algorithms (testing/development)
│   ├── kinematics.py                  # Forward kinematics
│   ├── models.py                      # Robot and operator models
│   ├── sensors.py                     # Sensor data structures
│   ├── trajectory.py                  # Trajectory generation
│   │
│   ├── drivers/                       # Hardware drivers
│   │   ├── __init__.py
│   │   ├── null_driver.py             # Simulation driver
│   │   └── serial_arduino.py          # Arduino serial driver
│   │
│   ├── network/                       # Network communication
│   │   ├── __init__.py
│   │   ├── bluetooth_receiver.py     # Bluetooth serial receiver
│   │   ├── protocol.py                # Packet serialization
│   │   └── receiver.py                # Receiver factory
│   │
│   ├── safety/                        # Safety systems
│   │   ├── __init__.py
│   │   └── checker.py                 # Safety validation
│   │
│   └── teleoperation/                 # Teleoperation control
│       ├── __init__.py
│       ├── controller.py              # Main teleoperation controller
│       ├── integrator.py              # Velocity integration
│       └── mapper.py                   # Operator-to-robot mapping
│
├── tests/                             # Test suite
│   ├── test_bluetooth.py              # Bluetooth component tests
│   ├── test_control.py                # Control algorithm tests
│   ├── test_ik.py                     # Inverse kinematics tests
│   ├── test_kinematics.py             # Forward kinematics tests
│   ├── test_models.py                 # Model loading tests
│   ├── test_serial.py                 # Serial communication tests
│   ├── test_teleoperation.py          # Teleoperation system tests
│   └── test_trajectory.py             # Trajectory generation tests
│
├── examples/                          # Example scripts
│   └── smoke_test.py                  # Basic functionality test
│
├── scripts/                           # Setup and automation scripts // look here for when it not seting up right also it should you bash
│   ├── setup_bluetooth_pi.sh         # Raspberry Pi Bluetooth setup
│   └── telearm-bluetooth.service     # Systemd service for Bluetooth // why not just make one and call it set up 
│
├── ros2/                              # ROS 2 integration (optional)
│   ├── __init__.py
│   ├── package.xml                    # ROS 2 package metadata
│   ├── setup.cfg                      # ROS 2 setup configuration
│   ├── setup.py                       # ROS 2 package setup // but then we have set up here on the ros2 which is the py system
│   │
│   ├── launch/                        # ROS 2 launch files
│   │   └── telearm_rviz.launch.py     # RViz visualization launch
│   │
│   ├── rviz/                          # RViz configuration
│   │   └── telearm.rviz               # Robot visualization config
│   │
│   ├── urdf/                          # Robot description files
│   │   └── telearm.urdf               # Robot model description
│   │
│   ├── resource/                      # ROS 2 resources
│   │   └── telearm_ros2               # Package resource file
│   │
│   └── telearm_ros2/                  # ROS 2 package code
│       ├── __init__.py
│       └── joint_state_bridge.py     # ROS 2 joint state publisher
│
├── progress-doc-wifi/                 # WiFi-specific documentation
│   ├── teleoperation-wifi.md          # WiFi teleoperation details
│   ├── teleoperation.md               # General teleoperation guide
│   ├── wifi-instructions.md           # WiFi setup instructions
│   ├── wifi-knowledge.md              # WiFi technical knowledge
│   └── cleanup-summary.md             # Codebase cleanup documentation
│
└── telearm.egg-info/                  # Python package metadata (generated)
    ├── dependency_links.txt
    ├── entry_points.txt
    ├── PKG-INFO
    ├── requires.txt
    ├── SOURCES.txt
    └── top_level.txt
```

## Core Package Structure (`telearm/`)

```
telearm/
├── __init__.py                        # Package exports and version
├── cli.py                             # Command-line interface
│   ├── cmd_home()                     # Move to home position
│   ├── cmd_move()                     # Move to Cartesian position
│   ├── cmd_joints()                   # Move to joint positions
│   ├── cmd_status()                   # Show system status
│   ├── cmd_calibrate()                # Run calibration
│   └── cmd_teleop()                   # Start teleoperation
│
├── models.py                          # Robot and operator models
│   ├── ArmModel                       # Robot arm model class
│   ├── OperatorArmModel               # Human arm model class
│   ├── JointSpec                      # Joint specification
│   ├── JointLimit                     # Joint limit constraints
│   ├── DH                             # Denavit-Hartenberg parameters
│   ├── load_from_config()             # Load robot model from YAML
│   └── load_operator_from_config()    # Load operator model from YAML
│
├── kinematics.py                      # Forward kinematics
│   ├── forward_kinematics()           # Compute end-effector pose
│   ├── jacobian()                     # Compute Jacobian matrix
│   └── _dh_transform()                # DH transformation matrix
│
├── ik.py                              # Inverse kinematics
│   ├── inverse_kinematics()           # Damped least squares IK
│   ├── _damped_least_squares()        # DLS algorithm
│   └── _clamp_joints()                # Apply joint limits
│
├── trajectory.py                      # Trajectory generation
│   ├── cubic_time_scaling()           # Cubic time scaling
│   ├── quintic_time_scaling()         # Quintic time scaling
│   └── _cubic_polynomial()            # Cubic polynomial coefficients
│
├── control.py                         # Motion control
│   ├── MotionController               # Main motion controller
│   ├── _execute_trajectory()          # Execute trajectory
│   └── _send_command()                # Send servo command
│
├── sensors.py                         # Sensor data structures
│   ├── IMUReading                     # Raw IMU data
│   ├── Orientation                    # Processed orientation
│   ├── OperatorPose                   # Operator pose estimation
│   ├── TeleopPacket                   # Network packet format
│   ├── create_mock_imu_reading()      # Mock IMU data
│   └── create_mock_operator_pose()    # Mock operator pose
│
├── imu_fusion.py                      # IMU fusion algorithms
│   ├── ComplementaryFilter            # Quaternion-based orientation estimation
│   ├── OperatorPoseEstimator          # Multi-IMU pose estimation
│   ├── FilterState                    # Filter state management
│   ├── create_mock_imu_fusion_data()  # Mock data generation
│   └── validate_imu_fusion_algorithm() # Algorithm validation
│
├── drivers/                           # Hardware abstraction
│   ├── __init__.py
│   ├── ServoDriver                    # Abstract servo driver
│   ├── NullServoDriver                # Simulation driver
│   └── SerialArduinoDriver            # Arduino serial driver
│
├── network/                           # Network communication
│   ├── __init__.py
│   ├── TeleopProtocol                 # Packet serialization
│   ├── NetworkStats                   # Network statistics
│   ├── OperatorDataReceiver           # WiFi UDP receiver
│   ├── BluetoothOperatorDataReceiver  # Bluetooth serial receiver
│   ├── MockOperatorDataReceiver       # Mock receiver for testing
│   └── create_receiver()              # Receiver factory function
│
├── safety/                            # Safety systems
│   ├── __init__.py
│   ├── SafetyChecker                  # Safety validation
│   ├── EmergencyStopHandler           # Emergency stop handling
│   ├── _check_joint_limits()          # Joint limit validation
│   ├── _check_workspace()             # Workspace validation
│   └── _check_emergency_stop()        # Emergency stop check
│
└── teleoperation/                     # Teleoperation control
    ├── __init__.py
    ├── TeleopController               # Main teleoperation controller
    ├── VelocityMapper                 # Operator-to-robot mapping
    ├── VelocityIntegrator             # Velocity integration
    ├── SmoothVelocityIntegrator       # Smooth velocity integration
    ├── _control_loop()                # Main control loop
    ├── _process_operator_data()       # Process operator packets
    └── _update_robot_state()          # Update robot state
```

## Configuration Structure (`config/`)

```
config/
├── robot.yaml                         # Robot model configuration
│   ├── robot:                         # Robot metadata
│   │   ├── name: "telearm-5dof"       # Robot name
│   │   └── dof: 5                     # Degrees of freedom
│   ├── joints:                        # Joint specifications
│   │   ├── name: "base_yaw"           # Joint name
│   │   ├── home: 0.0                  # Home position (rad)
│   │   ├── limit_min: -180.0          # Minimum limit (deg)
│   │   ├── limit_max: 180.0           # Maximum limit (deg)
│   │   ├── max_velocity: 1.0          # Max velocity (rad/s)
│   │   └── max_acceleration: 2.0      # Max acceleration (rad/s²)
│   └── dh_parameters:                 # DH parameters
│       ├── a: 0.05                    # Link length (m)
│       ├── alpha: 90.0                # Link twist (deg)
│       ├── d: 0.10                    # Link offset (m)
│       └── theta: 0.0                 # Joint angle (rad)
│
├── pins.yaml                          # Hardware pin configuration
│   ├── hardware:                      # Hardware settings
│   │   └── arduino:                   # Arduino configuration
│   │       ├── baud_rate: 115200      # Serial baud rate
│   │       └── default_port: "/dev/ttyUSB0"  # Serial port
│   └── servos:                        # Servo configuration
│       └── pins: [3, 5, 6, 9, 10]     # Servo signal pins
│
├── teleop.yaml                        # Teleoperation configuration
│   ├── teleoperation:                 # Main teleoperation settings
│   │   ├── mode: "wifi"               # Communication mode
│   │   ├── velocity_scale: 0.6        # Velocity scaling factor
│   │   ├── update_rate_hz: 100        # Control loop frequency
│   │   ├── timeout_ms: 200            # Connection timeout
│   │   ├── port: 5000                 # WiFi UDP port
│   │   └── bluetooth:                 # Bluetooth settings
│   │       ├── port: "/dev/rfcomm0"   # Bluetooth serial port/////////////////////////////////////////////////////////////////////////////////////////error/////////////////////////////////////////////////////////////
│   │       └── baud_rate: 115200      # Bluetooth baud rate
│   │   ├── mapping:                   # Mapping configuration
│   │   │   ├── type: "velocity_based" # Mapping type
│   │   │   └── null_space_gain: 0.1   # Null space gain
│   │   └── safety:                    # Safety configuration
│   │       ├── enforce_joint_limits: true
│   │       ├── enforce_workspace: true
│   │       ├── soft_limit_margin_deg: 5.0
│   │       └── emergency_stop_pin: 13
│
└── operator_arm.yaml                  # Operator model configuration
    ├── operator:                      # Operator metadata
    │   ├── name: "human_arm"          # Operator name
    │   └── dof: 3                     # Degrees of freedom
    └── joints:                        # Operator joint specifications
        ├── name: "shoulder_pitch"      # Joint name
        ├── home: 0.0                  # Home position (rad)
        ├── limit_min: -90.0            # Minimum limit (deg)
        ├── limit_max: 90.0             # Maximum limit (deg)
        ├── max_velocity: 2.0          # Max velocity (rad/s)
        └── max_acceleration: 4.0      # Max acceleration (rad/s²)
```

## Firmware Structure (`firmware/`)

```
firmware/
├── arduino/                           # Arduino Uno firmware
│   └── telearm_driver/
│       └── telearm_driver.ino         # Servo control firmware
│           ├── setup()                # Initialize servos and serial // overloaded operators
│           ├── loop()                 # Main control loop
│           ├── parseCommand()         # Parse incoming commands
│           ├── moveServo()            # Move individual servo
│           ├── emergencyStop()        # Emergency stop handler
│           └── watchdog()             # Watchdog timer
│
└── esp32/                             # ESP32 operator tracker firmware
    └── operator_tracker/
        └── operator_tracker.ino       # IMU fusion and communication
            ├── setup()                # Initialize IMUs and communication // overloaded operators
            ├── loop()                 # Main control loop
            ├── initializeIMUs()       # Initialize MPU-9250 sensors
            ├── connectToWiFi()        # Connect to WiFi network
            ├── setupBluetooth()       # Initialize Bluetooth
            ├── updateIMUFusion()      # Update IMU fusion (200Hz)
            ├── transmitOperatorPose() # Transmit pose data (100Hz)
            ├── updateComplementaryFilter() # IMU fusion algorithm
            └── calculateJointAngles() # Estimate joint angles
```

## Test Structure (`tests/`)

```
tests/
├── test_bluetooth.py                  # Bluetooth component tests
│   ├── TestBluetoothOperatorDataReceiver  # Bluetooth receiver tests
│   ├── test_bluetooth_receiver_creation() # Test receiver creation
│   ├── test_bluetooth_packet_reception() # Test packet reception
│   ├── test_bluetooth_timeout_handling() # Test timeout handling
│   └── test_receiver_factory_bluetooth() # Test factory function
│
├── test_control.py                    # Control algorithm tests
│   ├── TestMotionController           # Motion controller tests
│   ├── test_home_position()           # Test home position
│   ├── test_trajectory_execution()    # Test trajectory execution
│   └── test_emergency_stop()          # Test emergency stop
│
├── test_ik.py                         # Inverse kinematics tests
│   ├── TestInverseKinematics          # IK solver tests
│   ├── test_ik_solution()             # Test IK solutions
│   ├── test_joint_limits()            # Test joint limit handling
│   └── test_singularity_handling()    # Test singularity handling
│
├── test_kinematics.py                # Forward kinematics tests
│   ├── TestForwardKinematics          # FK computation tests
│   ├── test_fk_calculation()          # Test FK calculations
│   ├── test_jacobian()                # Test Jacobian computation
│   └── test_dh_parameters()           # Test DH parameter usage
│
├── test_models.py                     # Model loading tests
│   ├── TestRobotModel                 # Robot model tests
│   ├── TestOperatorModel              # Operator model tests
│   ├── test_config_loading()          # Test config file loading
│   └── test_model_validation()        # Test model validation
│
├── test_serial.py                     # Serial communication tests
│   ├── TestSerialArduinoDriver        # Serial driver tests
│   ├── test_serial_connection()       # Test serial connection
│   ├── test_command_parsing()         # Test command parsing
│   └── test_error_handling()          # Test error handling
│
├── test_teleoperation.py              # Teleoperation system tests
│   ├── TestTeleopController           # Teleoperation controller tests
│   ├── TestVelocityMapper             # Velocity mapping tests
│   ├── TestVelocityIntegrator         # Velocity integration tests
│   └── TestSafetyChecker              # Safety system tests
│
└── test_trajectory.py                 # Trajectory generation tests
    ├── TestTrajectoryGeneration       # Trajectory tests
    ├── test_cubic_scaling()           # Test cubic time scaling
    ├── test_quintic_scaling()         # Test quintic time scaling
    └── test_trajectory_validation()   # Test trajectory validation
```

## Documentation Structure (`docs/`)

```
docs/
├── setup.md                           # Complete setup guide
│   ├── Prerequisites                  # System requirements
│   ├── Software Installation          # Package installation
│   ├── Hardware Setup                 # Physical assembly
│   ├── Firmware Setup                 # Microcontroller programming
│   ├── Configuration Setup            # Configuration files
│   ├── Step-by-Step Setup Process     # Detailed setup procedure
│   ├── Communication Mode Setup       # WiFi/Bluetooth setup
│   ├── Testing                        # System testing
│   └── Troubleshooting                # Common issues
│
├── wiring.md                          # Hardware wiring guide
│   ├── ESP32 Operator Tracker         # ESP32 wiring
│   ├── Arduino Servo Driver           # Arduino wiring
│   ├── Raspberry Pi Connections       # Pi connections
│   ├── Power Supply Requirements      # Power specifications
│   └── Safety Considerations         # Electrical safety
│
├── safety.md                          # Safety procedures
│   ├── Safety Guidelines              # General safety rules
│   ├── Emergency Procedures           # Emergency stop procedures
│   ├── Hardware Safety                # Hardware safety checks
│   ├── Software Safety                # Software safety features
│   └── Operational Safety            # Operation safety guidelines
│
├── bluetooth-setup.md                 # Bluetooth configuration
│   ├── Hardware Overview              # Bluetooth hardware
│   ├── ESP32 Configuration            # ESP32 Bluetooth setup
│   ├── Raspberry Pi Setup             # Pi Bluetooth setup
│   ├── Pairing Process                # Device pairing
│   ├── Testing and Validation         # Bluetooth testing
│   └── Troubleshooting                # Bluetooth issues
│
├── program-overview.md                # System architecture
│   ├── System Architecture Overview   # High-level architecture
│   ├── File Structure and Interactions # File organization
│   ├── Data Flow and Packet Structure # Data flow
│   ├── Operational Procedures         # Operation procedures
│   ├── Troubleshooting Guide          # System troubleshooting
│   ├── Maintenance Procedures         # System maintenance
│   └── Advanced Configuration         # Advanced settings
│
├── data-hierarchy.md                  # Data structure documentation
│   ├── System Data Flow               # Data flow diagrams
│   ├── Data Structure Hierarchy       # Data structures
│   ├── Data Processing Pipeline       # Processing chain
│   ├── Data Storage Hierarchy         # Storage organization
│   ├── Data Validation Hierarchy      # Validation layers
│   └── Performance Metrics Hierarchy # Performance metrics
│
└── file-hierarchy.md                  # This file
    ├── Project Root Structure         # Top-level organization
    ├── Core Package Structure         # Python package structure
    ├── Configuration Structure        # Config file organization
    ├── Firmware Structure             # Firmware organization
    ├── Test Structure                 # Test organization
    └── Documentation Structure        # Documentation organization
```

## Build and Development Structure

```
Build Artifacts (generated)
├── build/                             # Build directory
├── dist/                              # Distribution packages
├── *.egg-info/                        # Python package metadata
├── .pytest_cache/                     # Test cache
├── .coverage                          # Test coverage data
├── htmlcov/                           # Coverage HTML reports
└── __pycache__/                       # Python bytecode cache

Development Tools
├── Makefile                           # Build commands
│   ├── install                        # Install package
│   ├── dev                            # Install dev dependencies
│   ├── test                           # Run tests
│   ├── lint                           # Run linting
│   ├── format                         # Format code
│   └── clean                          # Clean build artifacts
│
├── pyproject.toml                     # Python project configuration
│   ├── [build-system]                 # Build system requirements
│   ├── [project]                      # Project metadata
│   ├── [project.optional-dependencies] # Optional dependencies
│   ├── [tool.pytest.ini_options]     # Test configuration
│   ├── [tool.black]                  # Code formatting
│   └── [tool.ruff]                    # Linting configuration
│
└── requirements.txt                   # Python dependencies
    ├── numpy>=1.21.0                  # Numerical computing
    ├── scipy>=1.7.0                   # Scientific computing
    ├── pyyaml>=6.0                    # YAML parsing
    ├── pyserial>=3.5                  # Serial communication
    └── pytest>=6.0                    # Testing framework
```

This file hierarchy provides a complete overview of the Telearm project structure, making it easy to navigate and understand the codebase organization.
