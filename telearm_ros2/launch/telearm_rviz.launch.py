from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('telearm_ros2')
    urdf_path = os.path.join(pkg, 'urdf', 'telearm.urdf')
    rviz_path = os.path.join(pkg, 'rviz', 'telearm.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', urdf_path])}],
        ),
        Node(
            package='telearm_ros2',
            executable='joint_state_bridge',
            name='joint_state_bridge'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path]
        )
    ])
