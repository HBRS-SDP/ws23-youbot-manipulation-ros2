"""
Copyright 2023 Bonn-Rhein-Sieg University

ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
"""


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os


def generate_launch_description():

    robot_name = 'youbot-brsu-2'

    # Get path to the xacro file of the robot
    youbot_model_xacro = os.path.join(get_package_share_directory('mir_hardware_config'), robot_name, 'urdf','robot.urdf.xacro')

    robot_description_config = Command([FindExecutable(name='xacro'),' ', youbot_model_xacro])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    youbot_node = Node(
        package='mir_youbot_manipulation',
        executable='youbot_manipulation',
        name='youbot_manipulation_node',
        output='both',
        parameters=[robot_description]
    )

    return LaunchDescription([robot_state_publisher, youbot_node])