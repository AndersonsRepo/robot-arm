"""
Telemanipulator Core Abstractions (v0)

5-DOF serial arm (2 DOF base, 1 DOF elbow, 2 DOF wrist).
This file defines minimal interfaces and a reference implementation you can
run now, with placeholders for your real DH params, joint limits, and drivers.

Dependencies (pip): numpy, scipy (optional, used here only for splines)
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol, Sequence, Tuple, Optional, Callable
import numpy as np

# -----------------------------
# Core data models
# -----------------------------

@dataclass(frozen=True)
class JointLimit:
    min: float  # radians
    max: float  # radians

@dataclass(frozen=True)
class JointSpec:
    name: str
    home: float
    limit: JointLimit
    max_vel: float = np.deg2rad(90)  # rad/s
    max_acc: float = np.deg2rad(180) # rad/s^2

@dataclass(frozen=True)
class DH:
    """Standard DH parameters for each joint.
    a: link length, alpha: link twist, d: link offset, theta: joint variable
    For revolute joints, theta is the variable; d is constant.
    """
    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0

@dataclass
class ArmModel:
    joints: Tuple[JointSpec, ...]
    dh: Tuple[DH, ...]

    def n(self) -> int:
        return len(self.joints)

# -----------------------------
# Hardware abstraction
# -----------------------------

class ServoDriver(Protocol):
    def move_to(self, idx: int, angle_rad: float, speed: Optional[float]=None) -> None: ...
    def angles(self) -> np.ndarray: ...   # current measured/estimated joint angles (rad)

class NullServoDriver:
    """Sim-only placeholder that just stores target angles."""
    def __init__(self, n: int):
        self._q = np.zeros(n)
    def move_to(self, idx: int, angle_rad: float, speed: Optional[float]=None) -> None:
        self._q[idx] = angle_rad
    def angles(self) -> np.ndarray:
        return self._q.copy()

# Optional: Serial-connected Arduino driver
try:
    import serial, time
except Exception:
    serial = None
    time = None

class SerialServoDriver:
    """Arduino serial driver. Sends ASCII 'M,<idx>,<angle_rad>
' and expects 'OK'.
    Use with the Arduino sketch further below.  Install with: pip install pyserial
    Args:
        n: number of joints
        port: serial device (e.g., '/dev/ttyUSB0', '/dev/tty.usbmodem14101', 'COM3')
        baud: must match the Arduino sketch
        timeout: read timeout seconds for 'OK' lines
        echo_ok: whether to read/ignore an 'OK' line after each command
    """
    def __init__(self, n:int, port:str="/dev/ttyUSB0", baud:int=115200, timeout:float=0.25, echo_ok:bool=True):
        if serial is None:
            raise ImportError("pyserial not installed; pip install pyserial")
        self._q = np.zeros(n)
        self._n = n
        self._echo_ok = echo_ok
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Arduino resets on open; small delay to allow bootloader
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def move_to(self, idx:int, angle_rad:float, speed:Optional[float]=None)->None:
        if idx < 0 or idx >= self._n:
            raise IndexError("joint index out of range")
        self._q[idx] = angle_rad
        cmd = f"M,{idx},{angle_rad:.6f}
".encode()
        self.ser.write(cmd)
        if self._echo_ok:
            _ = self.ser.readline()  # consume 'OK' or error line

    def angles(self)->np.ndarray:
        # Until encoders/IMU are integrated, we return the last commanded values
        return self._q.copy()

# -----------------------------
# Kinematics
# -----------------------------

class Kinematics:
    def __init__(self, model: ArmModel):
        assert model.n() == len(model.dh)
        self.m = model

    @staticmethod
    def _dh_mat(a, alpha, d, theta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ ct, -st*ca,  st*sa, a*ct],
            [ st,  ct*ca, -ct*sa, a*st],
            [  0,     sa,     ca,    d],
            [  0,      0,      0,    1],
        ])

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """Forward kinematics: returns 4x4 pose of end-effector in base frame."""
        T = np.eye(4)
        for qi, dh in zip(q, self.m.dh):
            T = T @ self._dh_mat(dh.a, dh.alpha, dh.d, qi + dh.theta_offset)
        return T

    def jacobian_geometric(self, q: Sequence[float]) -> np.ndarray:
        """Geometric Jacobian (6xn) for a chain of revolute joints."""
        n = self.m.n()
        T = np.eye(4)
        origins = [T[:3, 3]]
        z_axes  = [T[:3, 2]]
        Ts = [T]
        # forward pass, store frames
        for qi, dh in zip(q, self.m.dh):
            T = T @ self._dh_mat(dh.a, dh.alpha, dh.d, qi + dh.theta_offset)
            Ts.append(T)
            origins.append(T[:3, 3])
            z_axes.append(T[:3, 2])
        J = np.zeros((6, n))
        pe = origins[-1]
        for i in range(n):
            zi = z_axes[i]
            pi = origins[i]
            Jv = np.cross(zi, pe - pi)
            Jw = zi
            J[:3, i] = Jv
            J[3:, i] = Jw
        return J

# -----------------------------
# Inverse kinematics (damped least squares)
# -----------------------------

@dataclass
class IKOptions:
    max_iters: int = 200
    tol_pos: float = 1e-3
    tol_rot: float = 1e-2
    lambda2: float = 1e-4  # damping^2

class IK:
    def __init__(self, kin: Kinematics):
        self.kin = kin

    @staticmethod
    def _log_SO3(R: np.ndarray) -> np.ndarray:
        """Axis-angle vector from rotation matrix (vee(log(R)))."""
        cos_theta = (np.trace(R) - 1)/2
        cos_theta = np.clip(cos_theta, -1, 1)
        theta = np.arccos(cos_theta)
        if theta < 1e-9:
            return np.zeros(3)
        w = (1/(2*np.sin(theta))) * np.array([
            R[2,1] - R[1,2],
            R[0,2] - R[2,0],
            R[1,0] - R[0,1],
        ])
        return theta * w

    def solve(self, q0: np.ndarray, T_target: np.ndarray, opts: IKOptions = IKOptions()) -> np.ndarray:
        q = q0.copy()
        for _ in range(opts.max_iters):
            T = self.kin.fk(q)
            p, R = T[:3,3], T[:3,:3]
            pt, Rt = T_target[:3,3], T_target[:3,:3]
            ep = pt - p
            eR = self._log_SO3(R.T @ Rt)
            e = np.concatenate([ep, eR])
            if np.linalg.norm(ep) < opts.tol_pos and np.linalg.norm(eR) < opts.tol_rot:
                break
            J = self.kin.jacobian_geometric(q)
            JJ = J @ J.T + opts.lambda2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJ, e)
            q = q + dq
        return q

# -----------------------------
# Trajectory generation (cubic time scaling between waypoints)
# -----------------------------

class Trajectory:
    @staticmethod
    def cubic_time_scaling(t0: float, tf: float, p0: np.ndarray, pf: np.ndarray) -> Callable[[float], np.ndarray]:
        dt = tf - t0
        if dt <= 0: raise ValueError("tf must be greater than t0")
        a0 = p0
        a3 = 2*(p0 - pf) / dt**3
        a2 = 3*(pf - p0) / dt**2
        def s(t: float) -> np.ndarray:
            tau = np.clip((t - t0)/dt, 0.0, 1.0)
            return a0 + a2*(tau*dt)**2 + a3*(tau*dt)**3
        return s

# -----------------------------
# Motion controller
# -----------------------------

class MotionController:
    def __init__(self, model: ArmModel, driver: ServoDriver):
        self.model = model
        self.driver = driver
        self.kin = Kinematics(model)
        self.ik = IK(self.kin)

    def go_home(self):
        for i, js in enumerate(self.model.joints):
            self.driver.move_to(i, js.home)

    def move_cartesian(self, q_start: np.ndarray, T_goal: np.ndarray, seconds: float = 2.0, steps: int = 50):
        T0 = self.kin.fk(q_start)
        p0, R0 = T0[:3,3], T0[:3,:3]
        pf, Rf = T_goal[:3,3], T_goal[:3,:3]
        # linear in position, slerp-ish via log/exp incremental (small angle per step)
        pos_traj = Trajectory.cubic_time_scaling(0.0, seconds, p0, pf)
        q = q_start.copy()
        for k in range(steps+1):
            t = seconds * k/steps
            pt = pos_traj(t)
            # simple orientation interpolation via error toward target
            T = self.kin.fk(q)
            Rt = Rf  # keep fixed for v0
            Tt = np.eye(4); Tt[:3,:3] = Rt; Tt[:3,3] = pt
            q = self.ik.solve(q, Tt)
            # enforce joint limits
            for i, js in enumerate(self.model.joints):
                q[i] = np.clip(q[i], js.limit.min, js.limit.max)
                self.driver.move_to(i, q[i])
        return q

# -----------------------------
# Example: define a 5-DOF model (placeholder numbers!)
# -----------------------------

def example_model() -> ArmModel:
    joints = (
        JointSpec("base_yaw",   home=0.0, limit=JointLimit(np.deg2rad(-180), np.deg2rad(180))),
        JointSpec("base_pitch", home=0.0, limit=JointLimit(np.deg2rad(-90),  np.deg2rad(90))),
        JointSpec("elbow",      home=0.0, limit=JointLimit(np.deg2rad(-135), np.deg2rad(135))),
        JointSpec("wrist_pitch",home=0.0, limit=JointLimit(np.deg2rad(-135), np.deg2rad(135))),
        JointSpec("wrist_roll", home=0.0, limit=JointLimit(np.deg2rad(-180), np.deg2rad(180))),
    )
    # Replace with measured DH once assembled
    dh = (
        DH(a=0.05, alpha=np.deg2rad(90),  d=0.10),
        DH(a=0.15, alpha=0.0,             d=0.00),
        DH(a=0.12, alpha=0.0,             d=0.00),
        DH(a=0.05, alpha=np.deg2rad(90),  d=0.00),
        DH(a=0.02, alpha=0.0,             d=0.00),
    )
    return ArmModel(joints=joints, dh=dh)

# -----------------------------
# Arduino sketch (save as arduino/telearm_driver/telearm_driver.ino)
# -----------------------------
# The sketch below drives hobby servos from simple serial commands.
# Protocol from host (Python):  M,<joint_idx>,<angle_rad>

# /*
# #include <Servo.h>
# 
# constexpr int NUM_JOINTS = 5;
# int PINS[NUM_JOINTS] = {3, 5, 6, 9, 10};
# Servo servos[NUM_JOINTS];
# 
# // Tune these per joint/servo
# int   us_min[NUM_JOINTS] = {500, 500, 500, 500, 500};
# int   us_max[NUM_JOINTS] = {2500,2500,2500,2500,2500};
# float rad_min[NUM_JOINTS]= {-3.1416, -1.5708, -2.3562, -2.3562, -3.1416};
# float rad_max[NUM_JOINTS]= { 3.1416,  1.5708,  2.3562,  2.3562,  3.1416};
# 
# String buf;
# 
# int clampi(int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }
# float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }
# 
# int radToUs(int i, float rad){
#   rad = clampf(rad, rad_min[i], rad_max[i]);
#   float t = (rad - rad_min[i]) / (rad_max[i] - rad_min[i]); // 0..1
#   int us = (int)(us_min[i] + t * (us_max[i] - us_min[i]));
#   return clampi(us, us_min[i], us_max[i]);
# }
# 
# void setup(){
#   Serial.begin(115200);
#   for(int i=0;i<NUM_JOINTS;i++){
#     servos[i].attach(PINS[i]);
#     servos[i].writeMicroseconds((us_min[i]+us_max[i])/2); // mid
#   }
#   Serial.println("READY");
# }
# 
# void loop(){
#   while (Serial.available()){
#     char c = (char)Serial.read();
#     if (c == '
'){
#       if (buf.length() > 0 && buf.charAt(0) == 'M'){
#         int p1 = buf.indexOf(',');
#         int p2 = buf.indexOf(',', p1+1);
#         int p3 = buf.indexOf(',', p2+1);
#         int idx = buf.substring(p1+1, p2).toInt();
#         float ang = buf.substring(p2+1, (p3==-1?buf.length():p3)).toFloat();
#         if (idx >=0 && idx < NUM_JOINTS){
#           int us = radToUs(idx, ang);
#           servos[idx].writeMicroseconds(us);
#           Serial.println("OK");
#         } else {
#           Serial.println("ERR");
#         }
#       }
#       buf = "";
#     } else if (c != '
'){
#       buf += c;
#     }
#   }
# }
# */

# -----------------------------
# Smoke test
# -----------------------------

if __name__ == "__main__":
    model = example_model()
    driver = NullServoDriver(model.n())
    ctrl = MotionController(model, driver)
    ctrl.go_home()

    q0 = driver.angles()
    # target: move 5 cm in x, 4 cm in z
    T_goal = np.eye(4)
    T_goal[:3,3] = np.array([0.05, 0.0, 0.04])
    qf = ctrl.move_cartesian(q_start=q0, T_goal=T_goal, seconds=2.0, steps=40)
    print("Final q (rad):", qf)
    print("EE pose:\n", ctrl.kin.fk(qf))
