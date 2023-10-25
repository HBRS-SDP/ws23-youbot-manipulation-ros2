import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import launch_ros

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'youbot_base.urdf'
    #xacro_file = "box_bot.xacro"
    package_description = "youbot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        # parameters=[{'use_sim_time': True, 'robot_description': open(robot_desc_path).read()}],
        parameters=[{'use_sim_time': True, 'robot_description': launch_ros.parameter_descriptions.ParameterValue(value=Command(['xacro ',robot_desc_path]), value_type=str)}],
        # parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ',robot_desc_path])}],
        output="screen"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rviz.rviz')


    # rviz_node = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         output='screen',
    #         name='rviz_node',
    #         parameters=[{'use_sim_time': True}],
    #         arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            # rviz_node
        ]
    )
