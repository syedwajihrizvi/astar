import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    pkg_project_bringup = get_package_share_directory(
        'astar_bringup'
    )

    pkg_turtlebot_description = get_package_share_directory(
        'turtlebot3_description'
    )

    urdf_file = os.path.join(
        pkg_turtlebot_description,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    robot_desc = xacro.process_file(
        urdf_file,
        mappings={
            'namespace': ''
        }
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(
                pkg_project_bringup,
                'config',
                'turtlebot3.rviz'
            )
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        condition=IfCondition(
            LaunchConfiguration('rviz')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz.'
        ),

        robot_state_publisher,
        rviz
    ])