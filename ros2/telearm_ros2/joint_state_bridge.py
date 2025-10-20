import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
# Import teleoperation components
from telearm import load_from_config
from telearm.drivers.null_driver import NullServoDriver

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('telearm_joint_state_bridge')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0/30.0, self.on_timer)  # 30 Hz

        # Load robot model
        self.robot_model = load_from_config()
        self.driver = NullServoDriver(self.robot_model.n())
        
        # Initialize robot to home position
        self.q = np.array([js.home for js in self.robot_model.joints])

        # Create a small target pose and precompute the trajectory in joint space using IK steps.
        self.t = 0.0
        self.seconds = 4.0
        self.steps = 120
        T_goal = np.eye(4); T_goal[:3,3] = np.array([0.05, 0.0, 0.04])  # +5 cm X, +4 cm Z
        
        # Generate trajectory waypoints
        self.q_traj = []
        q = self.q.copy()
        T0 = self.robot_model.forward_kinematics(q)
        p0 = T0[:3,3]; pf = T_goal[:3,3]
        
        # Simple linear interpolation for demonstration
        for i in range(self.steps):
            t = i / (self.steps - 1)
            p_interp = p0 + t * (pf - p0)
            T_interp = T_goal.copy()
            T_interp[:3,3] = p_interp
            
            # Solve IK for this pose
            q_sol = self.robot_model.inverse_kinematics(T_interp, q)
            if q_sol is not None:
                self.q_traj.append(q_sol)
                q = q_sol  # Warm-start next iteration with latest solution
            else:
                self.q_traj.append(q)  # Keep previous solution if IK fails
        
        self.get_logger().info(f"Generated {len(self.q_traj)} trajectory waypoints")
        self.idx = 0

        # Prepare joint names (must match URDF)
        self.names = [js.name for js in self.robot_model.joints]
        self.get_logger().info('JointStateBridge ready (publishing /joint_states at 30 Hz)')

    def on_timer(self):
        # Loop playback
        self.q = self.q_traj[self.idx]
        self.idx = (self.idx + 1) % len(self.q_traj)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = self.q.tolist()
        msg.velocity = [0.0] * len(self.q)  # Zero velocity for visualization
        msg.effort = [0.0] * len(self.q)     # Zero effort for visualization
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()