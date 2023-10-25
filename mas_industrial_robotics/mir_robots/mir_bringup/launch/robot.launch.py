#!/usr/bin/env python3

"""

Copyright 2022 Bonn-Rhein-Sieg University

Author: Vamsi Kalagaturu

"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():

    # argument for robot arm only launch
    declare_arm_only_arg = DeclareLaunchArgument(
        'arm_only',
        default_value='false',
        description='Launch only robot arm')

    robot_name = os.environ['ROBOT']

    # planning_context
    youbot_xacro_file = os.path.join(get_package_share_directory('mir_hardware_config'), robot_name, 'urdf',
                                     'robot.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', youbot_xacro_file])

    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    robot_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_bringup'), 'robots',),
            f'/youbot-brsu-common.launch.py']),
        condition=UnlessCondition(LaunchConfiguration('arm_only'))
    )

    robot_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_bringup'), 'robots',),
            f'/youbot-brsu-arm.launch.py']),
        condition=IfCondition(LaunchConfiguration('arm_only'))
    )

    return LaunchDescription([
        declare_arm_only_arg,
        robot_state_publisher,
        robot_common_launch,
        robot_arm_launch
    ])
