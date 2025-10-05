# Telearm - 5-DOF Robot Arm Control System

A modular Python package for controlling a 5-degree-of-freedom serial robot arm with forward/inverse kinematics, motion control, and hardware abstraction.

## Package Structure

### Core Package (`telearm/`)
- **`models.py`** - Joint specifications, DH parameters, and arm models
- **`kinematics.py`** - Forward kinematics and Jacobian computation
- **`ik.py`** - Inverse kinematics solver using damped least squares
- **`trajectory.py`** - Cubic time scaling trajectory generation
- **`control.py`** - Motion controller with hardware abstraction
- **`drivers/`** - Hardware drivers (simulation and serial Arduino)

### ROS 2 Integration (`telearm_ros2/`)
- **`joint_state_bridge.py`** - Publishes joint states for RViz visualization
- **`launch/telearm_rviz.launch.py`** - Launch file for ROS 2 visualization
- **`urdf/telearm.urdf`** - URDF model for robot visualization
- **`rviz/telearm.rviz`** - RViz configuration

### Hardware (`hardware/`)
- **`arduino/telearm_driver/telearm_driver.ino`** - Arduino sketch for servo control

### Examples (`examples/`)
- **`smoke_test.py`** - Basic functionality test

## Quick Start

### 1. Test the Core Package
```bash
python3 -m examples.smoke_test
```

### 2. ROS 2 Visualization (requires ROS 2)
```bash
# Build the ROS 2 package
colcon build --packages-select telearm_ros2

# Launch visualization
ros2 launch telearm_ros2 telearm_rviz.launch.py
```

### 3. Hardware Setup
1. Upload `hardware/arduino/telearm_driver/telearm_driver.ino` to Arduino
2. Connect servos to pins 3, 5, 6, 9, 10
3. Use `SerialServoDriver` in your code

## Dependencies
- `numpy` - Mathematical operations
- `scipy` - Optional, for advanced trajectory generation
- `pyserial` - For Arduino communication (optional)
- `rclpy` - For ROS 2 integration (optional)

## Features
- **Forward Kinematics**: DH parameter-based pose computation
- **Inverse Kinematics**: Damped least squares solver
- **Motion Control**: Cartesian trajectory planning
- **Hardware Abstraction**: Simulation and real hardware drivers
- **ROS 2 Integration**: Joint state publishing and RViz visualization
- **Arduino Support**: Serial communication for servo control

## API Usage

```python
from telearm import example_model, NullServoDriver, MotionController
import numpy as np

# Create model and controller
model = example_model()
driver = NullServoDriver(model.n())
ctrl = MotionController(model, driver)

# Move to home position
ctrl.go_home()

# Plan Cartesian motion
q_start = driver.angles()
T_goal = np.eye(4)
T_goal[:3, 3] = [0.05, 0.0, 0.04]  # 5cm in X, 4cm in Z
q_final = ctrl.move_cartesian(q_start, T_goal, seconds=2.0)
```

## License
MIT