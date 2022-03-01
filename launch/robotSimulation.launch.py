import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('diff_robot'))
    xacro_file = os.path.join(pkg_path, 'description',
                              'diffRobotBasic.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(),
              'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      output='screen',
                                      parameters=[params])

    rviz_config_path = os.path.join(
        pkg_path, 'configuration', 'rviz2_diff_robot.rviz')
    node_rviz2 = Node(package='rviz2',
                      executable='rviz2',
                      output='log',
                      arguments=['-d', rviz_config_path])

    gazebo_launch_path = os.path.join(get_package_share_directory(
        'gazebo_ros'), 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path))

    gazebo_spawn = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot'])

    constant_vel = Node(package='diff_robot',
                        executable='const_velocity_node')

    odom_frame_tf = Node(package='tf2_ros',
                         executable='static_transform_publisher',
                         arguments=['0 0 0 0 0 0 odom base_link'])

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use sim time if true'),
        node_robot_state_publisher,
        gazebo,
        node_rviz2,
        gazebo_spawn,
        # constant_vel,
        odom_frame_tf
    ])
