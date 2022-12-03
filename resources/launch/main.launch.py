#!/usr/bin/env python3

from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    project_dir = path.abspath(path.dirname(path.dirname(__file__)))
    launch_file_dir = path.join(project_dir, 'launch')
    world_file_dir = path.join(project_dir, 'worlds')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_rviz = LaunchConfiguration('rviz', default='false')
    world_file_name = LaunchConfiguration('world', default='hexagon.world')
    model_name = LaunchConfiguration('model', default='burger')
    x_pose = LaunchConfiguration('x_pose', default='2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.5')

    world_path = PathJoinSubstitution([world_file_dir, world_file_name])

    declare_world_file_cmd = DeclareLaunchArgument(
        'world', default_value='hexagon.world',
        description='Specify the world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': model_name
        }.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(launch_file_dir, 'spawn_bot.launch.py')
        ),
        launch_arguments={
            'model': model_name,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(launch_file_dir, 'cartographer.launch.py')
        ),
        launch_arguments={
            'rviz': launch_rviz,
        }.items()
    )

    ld = LaunchDescription([
    	declare_world_file_cmd,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        cartographer_cmd,
    ])


    return ld

