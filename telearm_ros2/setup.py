from setuptools import setup
package_name = 'telearm_ros2'
setup(
  name=package_name,
  version='0.0.1',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/telearm_rviz.launch.py']),
    ('share/' + package_name + '/urdf', ['urdf/telearm.urdf']),
    ('share/' + package_name + '/rviz', ['rviz/telearm.rviz']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='You',
  maintainer_email='you@example.com',
  description='ROS 2 bridge for Telearm kinematics-only visualization',
  license='MIT',
  entry_points={
    'console_scripts': [
      'joint_state_bridge = telearm_ros2.joint_state_bridge:main',
    ],
  },
)
