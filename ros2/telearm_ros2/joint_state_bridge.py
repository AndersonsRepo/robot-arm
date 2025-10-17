import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
# Import your kinematics/controller from the split package:
from telearm import ArmModel, JointSpec, JointLimit, DH, MotionController, example_model
from telearm.drivers.null_driver import NullServoDriver

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('telearm_joint_state_bridge')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0/30.0, self.on_timer)  # 30 Hz

        # Build your model & controller
        self.model = example_model()
        self.driver = NullServoDriver(self.model.n())
        self.ctrl = MotionController(self.model, self.driver)

        # Home pose
        self.ctrl.go_home()
        self.q = self.driver.angles()

        # Create a small target pose and precompute the trajectory in joint space using IK steps.
        self.t = 0.0
        self.seconds = 4.0
        self.steps = 120
        T_goal = np.eye(4); T_goal[:3,3] = np.array([0.05, 0.0, 0.04])  # +5 cm X, +4 cm Z
        # Generate a list of q waypoints by asking the controller to move (but we *don't* drive hardware)
        self.q_traj = []
        q = self.q.copy()
        T0 = self.ctrl.kin.fk(q)
        p0 = T0[:3,3]; pf = T_goal[:3,3]
        pos_traj = self.ctrl.trajectory.cubic_time_scaling(0.0, self.seconds, p0, pf) if hasattr(self.ctrl, 'trajectory') else None
        # Fallback to linear positions if Trajectory not exposed; use controller loop instead:
        if pos_traj is None:
            pos = lambda tt: p0 + (pf - p0) * (tt / self.seconds)
        else:
            pos = pos_traj

        Rf = T_goal[:3,:3]
        for k in range(self.steps+1):
            tt = self.seconds * k / self.steps
            pt = pos(tt)
            Tt = np.eye(4); Tt[:3,:3] = Rf; Tt[:3,3] = pt
            q = self.ctrl.ik.solve(q, Tt)
            # clip to limits
            for i, js in enumerate(self.model.joints):
                q[i] = np.clip(q[i], js.limit.min, js.limit.max)
            self.q_traj.append(q.copy())
        self.idx = 0

        # Prepare joint names (must match URDF)
        self.names = [js.name for js in self.model.joints]
        # For convenience, stash current index
        self.get_logger().info('JointStateBridge ready (publishing /joint_states at 30 Hz)')

    def on_timer(self):
        # loop playback
        self.q = self.q_traj[self.idx]
        self.idx = (self.idx + 1) % len(self.q_traj)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [float(x) for x in self.q]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
