"""
Telemanipulator Core Package (v0)

5-DOF serial arm with forward/inverse kinematics, motion control, and hardware abstraction.
"""

from .models import JointLimit, JointSpec, DH, ArmModel, example_model, load_robot_config, load_from_config
from .kinematics import Kinematics
from .ik import IK, IKOptions
from .trajectory import Trajectory
from .control import MotionController
from .drivers import NullServoDriver, SerialServoDriver

__version__ = "0.1.0"
__all__ = [
    'JointLimit', 'JointSpec', 'DH', 'ArmModel', 'example_model', 'load_robot_config', 'load_from_config',
    'Kinematics', 'IK', 'IKOptions', 'Trajectory', 'MotionController',
    'NullServoDriver', 'SerialServoDriver'
]
