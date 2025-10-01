# Telearm 5‑DOF — Core Kinematics + Arduino Driver

A tiny, hardware‑agnostic core for a 5‑DOF serial telemanipulator (2‑DOF base, 1‑DOF elbow, 2‑DOF wrist). It cleanly separates **math/planning** (Python) from **I/O** (Arduino C++), so you can simulate today and swap in real hardware tomorrow.

---

## Why this repo

* **Stable architecture:** math stays the same whether you run in sim or on the real arm.
* **Swappable drivers:** plug in a `NullServoDriver` (simulation) or a `SerialServoDriver` (Arduino/HC‑05).
* **Minimal surface area:** one interface (`ServoDriver`) is the only thing hardware must implement.

> See the bill of materials in your project files (BOM Telemanipulator.pdf) for parts and power notes.

---

## Features

* **Models:** `JointSpec`, `JointLimit`, `DH`, `ArmModel`
* **Kinematics:** Forward Kinematics (FK), Geometric Jacobian
* **IK:** Damped Least‑Squares (DLS) with pose error (position + axis‑angle orientation)
* **Trajectory:** cubic time scaling utility (smooth start/stop)
* **Controller:** `MotionController` that iterates IK along a Cartesian path and enforces joint limits
* **Drivers:**

  * `NullServoDriver` — in‑memory angles (simulation)
  * `SerialServoDriver` — speaks simple ASCII over USB/Bluetooth to Arduino
* **Arduino sketch:** minimal motor I/O: parse `M,<joint_idx>,<angle_rad>\n`, convert to servo pulses, reply `OK`.

---

## Directory layout

```
telearm/
  __init__.py
  models.py           # JointLimit, JointSpec, DH, ArmModel
  kinematics.py       # Kinematics (FK, Jacobian)
  ik.py               # IK + IKOptions
  trajectory.py       # Trajectory utilities
  control.py          # MotionController
  drivers/
    __init__.py
    null_driver.py    # NullServoDriver
    serial_arduino.py # SerialServoDriver (pyserial)
  examples/
    smoke_test.py     # builds example_model, runs a small move
hardware/
  arduino/
    telearm_driver/
      telearm_driver.ino  # motor I/O sketch (Servo lib)
```

> If you prefer a single file to start, use `telemanipulator_core.py` (contains everything). The split layout above is recommended once you’re ready to grow.

---

## Requirements

* **Python** 3.10+ (3.12+ recommended)
* **NumPy** (required)
* **PySerial** (only if you use Arduino): `pip install pyserial`
* **SciPy** (optional, not required by v0)
* **Arduino IDE** (or Arduino CLI) to flash the sketch

### Install

```bash
# from your virtual environment
pip install numpy pyserial
```

---

## Quickstart (simulation)

```bash
# Option A: monolithic file
python telemanipulator_core.py

# Option B: split package
python -m telearm.examples.smoke_test
```

Expected: printed joint angles and end‑effector pose; no hardware required.

---

## Quickstart (real hardware via Arduino)

1. **Wire servos** to the Arduino pins you plan to use. Use a **separate, adequately rated servo power supply** and **common ground** with the Arduino. (Do not power multiple hobby servos from the Arduino 5V pin.)
2. Open `hardware/arduino/telearm_driver/telearm_driver.ino` in Arduino IDE and upload it:

   * Set **baud 115200**.
   * Adjust `PINS[]`, `us_min/us_max`, and `rad_min/rad_max` per joint.
3. On your host (laptop/RPi), install `pyserial` and pick the serial port:

   * macOS: `/dev/tty.usbmodem*` or `/dev/tty.usbserial*`
   * Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`
   * Windows: `COM3`, `COM4`, ...
4. In `examples/smoke_test.py`, create the `SerialServoDriver` and run.

### Serial protocol

```
Host → Arduino:  M,<joint_idx>,<angle_rad>\n
Examples:
M,0,0.523599\n    # joint 0 → +30°
M,3,-1.047198\n   # joint 3 → −60°

Arduino → Host:  OK\n   (or ERR) after execution
```

---

## Configuring your robot

### 1) Joint limits

* Set per‑joint `JointLimit(min,max)` in **radians** (e.g., ±180° = ±π rad).
* The controller clips to these every step.

### 2) DH parameters

Update `example_model()` (or your own model) with measured Denavit–Hartenberg parameters for each link:

* `a`  (link length)
* `alpha` (link twist)
* `d`  (link offset)
* `theta_offset` (mechanical zero → model zero correction)

### 3) Servo range mapping (Arduino)

* Tune `us_min/us_max` (typically 500–2500 µs) for your servos.
* Set `rad_min/rad_max` so the radians you command map correctly to safe pulses.

---

## Example (host Python)

```python
from telearm.models import ArmModel, JointSpec, JointLimit, DH
from telearm.control import MotionController
from telearm.drivers.serial_arduino import SerialServoDriver
import numpy as np

# build model (replace placeholder DH/limits)
joints = (
  JointSpec("base_yaw",0.0, JointLimit(-np.pi, np.pi)),
  JointSpec("base_pitch",0.0, JointLimit(-np.pi/2, np.pi/2)),
  JointSpec("elbow",0.0, JointLimit(-3*np.pi/4, 3*np.pi/4)),
  JointSpec("wrist_pitch",0.0, JointLimit(-3*np.pi/4, 3*np.pi/4)),
  JointSpec("wrist_roll",0.0, JointLimit(-np.pi, np.pi)),
)
dh = (
  DH(a=0.05, alpha=np.pi/2, d=0.10),
  DH(a=0.15, alpha=0.0,     d=0.00),
  DH(a=0.12, alpha=0.0,     d=0.00),
  DH(a=0.05, alpha=np.pi/2, d=0.00),
  DH(a=0.02, alpha=0.0,     d=0.00),
)
model = ArmModel(joints=joints, dh=dh)

# choose driver
# driver = NullServoDriver(model.n())                 # simulation
from telearm.drivers.serial_arduino import SerialServoDriver
driver = SerialServoDriver(model.n(), port="/dev/tty.usbmodemXXXX")  # set your port

ctrl = MotionController(model, driver)
ctrl.go_home()
q0 = driver.angles()

T_goal = np.eye(4)
T_goal[:3,3] = np.array([0.05, 0.00, 0.04])  # move +5cm X, +4cm Z
qf = ctrl.move_cartesian(q_start=q0, T_goal=T_goal, seconds=2.0, steps=40)
print("Final q:", qf)
```

---

## Calibration workflow (suggested)

1. **Mechanical zero:** set a repeatable home for each joint (marks/jigs).
2. **Pulse range:** for each servo, scan pulses (e.g., 500→2500 µs) to find safe endpoints (no stall/bind). Update `us_min/us_max`.
3. **Radians mapping:** decide physical radian limits and set `rad_min/rad_max` to match endpoints.
4. **DH measurement:** measure `a`, `alpha`, `d` carefully from your linkage geometry.
5. **Theta offsets:** if “home” is not model zero, set `theta_offset` for each joint.
6. **Verify FK:** compare measured end‑effector positions to `fk(q)` predictions and refine.

---

## Safety notes

* Use a **separate power supply** for servos; **common ground** with the Arduino.
* Add an **inline fuse** and a **physical E‑stop** (kill power to motors).
* Keep **joint limits conservative** until calibration is complete.
* Secure the arm before first moves; start at **low speed**.

---

## Troubleshooting

* **No serial port / timeout:** correct port? On macOS, try `/dev/tty.usbmodem*`; on Windows, check Device Manager.
* **Arduino resets on connect:** this is normal; wait ~2s after opening serial.
* **Jitter / brown‑outs:** servo supply too weak; check grounds; add capacitors.
* **Wrong direction / scale:** fix `rad_min/rad_max` or `theta_offset`.
* **Pose won’t reach:** IK may be at a singularity or target is out of reach; move closer or change approach vector.

---

## Roadmap

* Joint‑space trapezoidal/S‑curve profiles
* Encoder/IMU feedback → real `angles()`
* PyBullet/MuJoCo sim loader from URDF
* Basic collision checking
* ROS 2 driver and TF publisher

---

## License

TBD (MIT recommended).
