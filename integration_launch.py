"""
ROS2 Launch File: AirSim-OpenVINS Integration

Launches:
  1. AirSim-OpenVINS bridge node (Python)
  2. OpenVINS (ov_msckf run_subscribe_msckf)
  3. RViz2 for visualization (optional)

Prerequisites:
  - AirSim environment must be running
  - ArduPilot SITL must be running (if using ArduCopter vehicle)
  - OpenVINS workspace must be built and sourced

Usage:
  # Source the workspace first
  source /mnt/rnd/Shubham/openvins_ws/install/setup.bash

  # Launch the integration
  ros2 launch /mnt/rnd/Shubham/openvins_ws/integration/integration_launch.py

  # Or with custom parameters
  ros2 launch /mnt/rnd/Shubham/openvins_ws/integration/integration_launch.py \
      rviz_enable:=false camera_rate_hz:=30.0
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Path to OpenVINS config
OPENVINS_CONFIG_DIR = '/mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config'
BRIDGE_SCRIPT = '/mnt/rnd/Shubham/openvins_ws/integration/airsim_openvins_bridge.py'


launch_args = [
    DeclareLaunchArgument(
        name='airsim_ip',
        default_value='127.0.0.1',
        description='AirSim server IP address',
    ),
    DeclareLaunchArgument(
        name='vehicle_name',
        default_value='Copter',
        description='AirSim vehicle name (must match settings.json)',
    ),
    DeclareLaunchArgument(
        name='camera_name',
        default_value='front_center',
        description='AirSim camera name (must match settings.json)',
    ),
    DeclareLaunchArgument(
        name='imu_rate_hz',
        default_value='200.0',
        description='IMU polling rate in Hz',
    ),
    DeclareLaunchArgument(
        name='camera_rate_hz',
        default_value='30.0',
        description='Camera capture rate in Hz',
    ),
    DeclareLaunchArgument(
        name='rviz_enable',
        default_value='true',
        description='Enable RViz2 visualization',
    ),
    DeclareLaunchArgument(
        name='verbosity',
        default_value='INFO',
        description='OpenVINS verbosity: ALL, DEBUG, INFO, WARNING, ERROR, SILENT',
    ),
    DeclareLaunchArgument(
        name='bridge_delay',
        default_value='0.0',
        description='Delay (seconds) before starting bridge node',
    ),
    DeclareLaunchArgument(
        name='openvins_delay',
        default_value='3.0',
        description='Delay (seconds) before starting OpenVINS (wait for bridge)',
    ),
]


def launch_setup(context):
    nodes = []

    config_path = os.path.join(OPENVINS_CONFIG_DIR, 'estimator_config.yaml')

    if not os.path.isfile(config_path):
        return [
            LogInfo(msg=f'ERROR: OpenVINS config not found: {config_path}')
        ]

    if not os.path.isfile(BRIDGE_SCRIPT):
        return [
            LogInfo(msg=f'ERROR: Bridge script not found: {BRIDGE_SCRIPT}')
        ]

    # 1. AirSim-OpenVINS Bridge Node
    bridge_node = Node(
        package=None,  # Standalone Python script
        executable='python3',
        name='airsim_openvins_bridge',
        arguments=[BRIDGE_SCRIPT],
        parameters=[{
            'airsim_ip': LaunchConfiguration('airsim_ip'),
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'camera_name': LaunchConfiguration('camera_name'),
            'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
            'camera_rate_hz': LaunchConfiguration('camera_rate_hz'),
            'image_width': 640,
            'image_height': 480,
            'fov_degrees': 90.0,
        }],
        output='screen',
    )

    bridge_delay = float(LaunchConfiguration('bridge_delay').perform(context))
    if bridge_delay > 0:
        nodes.append(TimerAction(period=bridge_delay, actions=[bridge_node]))
    else:
        nodes.append(bridge_node)

    # 2. OpenVINS Node (delayed to allow bridge to start publishing)
    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        namespace='ov_msckf',
        name='ov_msckf',
        output='screen',
        parameters=[
            {'verbosity': LaunchConfiguration('verbosity')},
            {'use_stereo': False},
            {'max_cameras': 1},
            {'save_total_state': True},
            {'config_path': config_path},
            # Topic remapping via parameters
            {'topic_imu': '/imu'},
            {'topic_camera0': '/camera/image'},
        ],
    )

    openvins_delay = float(LaunchConfiguration('openvins_delay').perform(context))
    nodes.append(TimerAction(period=openvins_delay, actions=[openvins_node]))

    # 3. RViz2 (optional)
    try:
        from ament_index_python.packages import get_package_share_directory
        rviz_config = os.path.join(
            get_package_share_directory('ov_msckf'),
            'launch',
            'display_ros2.rviz'
        )
    except Exception:
        rviz_config = ''

    rviz_args = []
    if rviz_config and os.path.isfile(rviz_config):
        rviz_args = ['-d', rviz_config]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz_enable')),
        arguments=rviz_args + ['--ros-args', '--log-level', 'warn'],
    )
    nodes.append(TimerAction(period=openvins_delay + 1.0, actions=[rviz_node]))

    return nodes


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
