#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import sys
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Add the path to the `utils` folder
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml  # noqa: E402


def generate_robot_nodes(context):
    additional_nodes = []
    # Get the arguments from the launch configuration
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    # Include the existing example.launch.py file
    additional_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'launch', 'example.launch.py'
                ])
            ),
            launch_arguments={
                'robot_config_file': robot_config_file,
                'controller_name': 'joint_impedance_with_ik_example_controller',
            }.items(),
        )
    )

    # Load the robot configuration file
    configs = load_yaml(robot_config_file)

    for _, config in configs.items():
        robot_ip = config['robot_ip']
        namespace = config['namespace']
        load_gripper = config['load_gripper']
        use_fake_hardware = config['use_fake_hardware']
        fake_sensor_commands = config['fake_sensor_commands']
        use_rviz = config['use_rviz']

        # Define the additional nodes
        additional_nodes.append(
          IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_fr3_moveit_config'),
                            'launch',
                            'move_group.launch.py',
                        ]
                    )
                ]
            ),
            launch_arguments={
                'robot_ip': str(robot_ip),
                'namespace': str(namespace),
                'load_gripper': str(load_gripper),
                'use_fake_hardware': str(use_fake_hardware),
                'fake_sensor_commands': str(fake_sensor_commands),
                'use_rviz': str(use_rviz),
            }.items(),
          ),
        )
    return additional_nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments and add additional ones if needed
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        # Generate robot nodes
        OpaqueFunction(function=generate_robot_nodes),
    ])
