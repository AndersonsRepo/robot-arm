"""Command-line interface for telearm control."""
import argparse
import sys
import numpy as np
from pathlib import Path

from . import load_from_config, NullServoDriver, MotionController
from .drivers import SerialServoDriver


def create_controller(args):
    """Create motion controller with appropriate driver."""
    try:
        model = load_from_config()
    except FileNotFoundError:
        print("Warning: config/robot.yaml not found, using example model")
        from . import example_model
        model = example_model()
    
    if args.sim:
        driver = NullServoDriver(model.n())
        print("Using simulation driver (null)")
    else:
        port = args.port or "/dev/ttyUSB0"
        try:
            driver = SerialServoDriver(model.n(), port=port)
            print(f"Connected to Arduino on {port}")
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            print("Falling back to simulation driver")
            driver = NullServoDriver(model.n())
    
    return MotionController(model, driver)


def cmd_home(args):
    """Move to home position."""
    ctrl = create_controller(args)
    print("Moving to home position...")
    ctrl.go_home()
    print("Home position reached")


def cmd_move(args):
    """Move to Cartesian position."""
    ctrl = create_controller(args)
    
    # Create target transform matrix
    T_goal = np.eye(4)
    T_goal[:3, 3] = [args.x, args.y, args.z]
    
    print(f"Moving to position ({args.x}, {args.y}, {args.z})...")
    
    # Get current joint angles
    q_start = ctrl.driver.angles()
    
    # Plan and execute motion
    q_final = ctrl.move_cartesian(q_start, T_goal, seconds=2.0)
    
    print("Motion completed")
    print(f"Final joint angles (rad): {q_final}")
    print(f"Final joint angles (deg): {np.rad2deg(q_final)}")


def cmd_joints(args):
    """Move to joint angles."""
    ctrl = create_controller(args)
    
    if len(args.angles) != ctrl.model.n():
        print(f"Error: Expected {ctrl.model.n()} joint angles, got {len(args.angles)}")
        sys.exit(1)
    
    q_target = np.array(args.angles)
    
    print(f"Moving to joint angles: {q_target} rad ({np.rad2deg(q_target)} deg)")
    
    # Move each joint to target angle
    for i, angle in enumerate(q_target):
        ctrl.driver.move_to(i, angle)
    
    print("Joint motion completed")


def cmd_status(args):
    """Print current status."""
    ctrl = create_controller(args)
    
    # Get current joint angles
    q_current = ctrl.driver.angles()
    
    # Compute forward kinematics
    T_current = ctrl.kin.fk(q_current)
    pos = T_current[:3, 3]
    
    print("Current Status:")
    print(f"Joint angles (rad): {q_current}")
    print(f"Joint angles (deg): {np.rad2deg(q_current)}")
    print(f"End-effector position (m): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    
    # Check joint limits
    for i, (js, angle) in enumerate(zip(ctrl.model.joints, q_current)):
        if angle < js.limit.min or angle > js.limit.max:
            print(f"WARNING: Joint {i} ({js.name}) outside limits!")


def cmd_calibrate(args):
    """Run calibration routine."""
    ctrl = create_controller(args)
    
    print("Calibration routine:")
    print("1. Moving to home position...")
    ctrl.go_home()
    
    print("2. Testing joint limits...")
    for i, js in enumerate(ctrl.model.joints):
        print(f"  Joint {i} ({js.name}): {js.limit.min:.2f} to {js.limit.max:.2f} rad")
    
    print("3. Testing forward kinematics...")
    q_home = np.array([js.home for js in ctrl.model.joints])
    T_home = ctrl.kin.fk(q_home)
    print(f"  Home position: {T_home[:3, 3]}")
    
    print("Calibration completed")


def cmd_run(args):
    """Run gamepad control loop."""
    import signal
    import time
    
    print("Starting gamepad control mode...")
    print("Press Ctrl+C to stop")
    
    from .teleoperation.controller import PositionController
    
    # Create position-based controller
    controller = PositionController(
        config_path=args.config,
        use_sim=args.sim
    )
    
    def signal_handler(sig, frame):
        print("\nStopping controller...")
        controller.stop()
        sys.exit(0)
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Start controller
        controller.start()
        
        # Keep running until interrupted
        while True:
            time.sleep(1.0)
            
            # Print status every 10 seconds
            if int(time.time()) % 10 == 0:
                status = controller.get_status()
                print(f"Status: IK={status['ik_solves']}, "
                      f"failures={status['ik_failures']}, "
                      f"packets={status['packets_sent']}, "
                      f"scale={status['motion_scale']:.2f}")
    
    except KeyboardInterrupt:
        print("\nController stopped by user")
    except Exception as e:
        print(f"Controller error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.stop()


def cmd_teleop(args):
    """Start teleoperation mode (gamepad)."""
    # Alias for cmd_run for backward compatibility
    cmd_run(args)


def cmd_demo(args):
    """Run pre-programmed demo trajectory."""
    print("Running demo trajectory...")
    
    from . import load_from_config
    from .control import MotionController
    from .drivers import NullServoDriver
    import numpy as np
    import time
    
    model = load_from_config()
    
    if args.sim:
        driver = NullServoDriver(model.n())
        ctrl = MotionController(model, driver)
        
        # Demo: Move to a few waypoints using joint angles
        waypoints = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),  # Home
            np.array([0.5, 0.2, 0.0, 0.0, 0.0]),  # Extended
            np.array([0.3, 0.1, -0.3, 0.1, 0.0]),  # Curved
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),  # Back home
        ]
        
        print("Moving through demo waypoints...")
        for i, waypoint in enumerate(waypoints):
            print(f"Waypoint {i+1}/{len(waypoints)}")
            for j, angle in enumerate(waypoint):
                driver.move_to(j, angle)
            time.sleep(2.0)
    else:
        from .drivers.serial_esp32 import ESP32SerialDriver
        esp32_config = {"default_port": args.port or "/dev/ttyUSB0", "baud_rate": 115200, "timeout": 0.3}
        driver = ESP32SerialDriver(
            port=esp32_config["default_port"],
            baud=esp32_config["baud_rate"],
            timeout=esp32_config["timeout"],
            num_joints=model.n()
        )
        if not driver.connect():
            print("Failed to connect to ESP32")
            return
        
        # Demo: Move to a few waypoints using joint angles
        waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0],  # Home
            [30.0, 20.0, 0.0, 0.0, 0.0],  # Extended (degrees)
            [20.0, 10.0, -20.0, 10.0, 0.0],  # Curved
            [0.0, 0.0, 0.0, 0.0, 0.0],  # Back home
        ]
        
        print("Moving through demo waypoints...")
        for i, waypoint in enumerate(waypoints):
            print(f"Waypoint {i+1}/{len(waypoints)}")
            driver.send_joint_targets(waypoint)
            time.sleep(2.0)
        
        driver.disconnect()
    
    print("Demo complete")


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(description="Telearm CLI - Control your 5-DOF robot arm")
    parser.add_argument("--port", help="Serial port (e.g., /dev/ttyUSB0, COM3)")
    parser.add_argument("--sim", action="store_true", help="Use simulation driver (no hardware)")
    
    subparsers = parser.add_subparsers(dest="command", required=True, help="Available commands")
    
    # Home command
    subparsers.add_parser("home", help="Move to home position")
    
    # Move command
    move_parser = subparsers.add_parser("move", help="Move to Cartesian position (x y z in meters)")
    move_parser.add_argument("x", type=float, help="X coordinate (meters)")
    move_parser.add_argument("y", type=float, help="Y coordinate (meters)")
    move_parser.add_argument("z", type=float, help="Z coordinate (meters)")
    
    # Joints command
    joints_parser = subparsers.add_parser("joints", help="Move to joint angles (radians)")
    joints_parser.add_argument("angles", nargs="+", type=float, help="Joint angles in radians")
    
    # Status command
    subparsers.add_parser("status", help="Print current joint angles and end-effector pose")
    
    # Calibrate command
    subparsers.add_parser("calibrate", help="Run calibration routine")
    
    # Run command (gamepad control)
    run_parser = subparsers.add_parser("run", help="Run gamepad control loop")
    run_parser.add_argument("--config", default="config/teleop.yaml",
                           help="Teleoperation config file")
    run_parser.add_argument("--sim", action="store_true",
                           help="Use simulation mode (no hardware)")
    
    # Demo command
    demo_parser = subparsers.add_parser("demo", help="Run pre-programmed demo trajectory")
    demo_parser.add_argument("--sim", action="store_true",
                            help="Use simulation mode (no hardware)")
    
    # Teleop command (alias for run, for backward compatibility)
    teleop_parser = subparsers.add_parser("teleop", help="Start gamepad teleoperation (alias for run)")
    teleop_parser.add_argument("--config", default="config/teleop.yaml",
                              help="Teleoperation config file")
    teleop_parser.add_argument("--sim", action="store_true",
                              help="Use simulation mode (no hardware)")
    
    args = parser.parse_args()
    
    try:
        if args.command == "home":
            cmd_home(args)
        elif args.command == "move":
            cmd_move(args)
        elif args.command == "joints":
            cmd_joints(args)
        elif args.command == "status":
            cmd_status(args)
        elif args.command == "calibrate":
            cmd_calibrate(args)
        elif args.command == "run":
            cmd_run(args)
        elif args.command == "demo":
            cmd_demo(args)
        elif args.command == "teleop":
            cmd_teleop(args)
        else:
            parser.print_help()
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
