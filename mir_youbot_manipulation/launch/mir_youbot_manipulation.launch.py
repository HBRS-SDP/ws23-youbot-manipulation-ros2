"""
Copyright 2023 Bonn-Rhein-Sieg University

ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
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


def generate_description():

    youbot_model_xacro = os.path.join(
        get_package_share_directory("youbot_description"), "urdf", "youbot_arm", "arm_corrected_dynamics.urdf.xacro"
    )
    robot_description_config = Command([FindExecutable(name="xacro"), '', youbot_model_xacro])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    return LaunchDescription([robot_state_publisher])