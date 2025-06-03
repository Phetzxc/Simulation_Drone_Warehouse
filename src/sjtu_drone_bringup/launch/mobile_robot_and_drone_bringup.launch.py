#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     https://www.gnu.org/licenses/gpl-3.0.en.html

import os
import yaml
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
import xacro


def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]

    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )
    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]


def rviz_node_generator(context, rviz_path):
    fixed_frame_value = LaunchConfiguration('fixed_frame').perform(context)
    rviz_arguments = ['-d', rviz_path]

    if fixed_frame_value:
        rviz_arguments.extend(['--fixed-frame', fixed_frame_value])

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_arguments,
            output='screen',
        )
    ]


def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    bcr_bot_path = get_package_share_directory('bcr_bot')

    rviz_path = os.path.join(sjtu_drone_bringup_path, "rviz", "rviz.rviz")
    yaml_file_path = os.path.join(sjtu_drone_bringup_path, 'config', 'drone.yaml')

    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    # BCR Bot spawn setup
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='bcr_bot')
    xacro_path = join(bcr_bot_path, 'urdf', 'bcr_bot.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command([
                'xacro ', xacro_path,
                ' camera_enabled:=', camera_enabled,
                ' stereo_camera_enabled:=', stereo_camera_enabled,
                ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                ' sim_gazebo:=', "true",
                ' odometry_source:=', odometry_source,
                ' robot_namespace:=', robot_namespace,
            ])}
        ],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']),
            '-z', "0.28",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="keyboard", description="Type of controller: keyboard (default) or joystick"),
        DeclareLaunchArgument("fixed_frame", default_value='', description='If provided, sets the fixed frame in RViz.'),

        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="1.0"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="3.14"),

        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="3.14"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),

        OpaqueFunction(
            function=rviz_node_generator,
            kwargs={'rviz_path': rviz_path},
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'sjtu_drone_gazebo.launch.py')
            ),
            launch_arguments={
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "roll": LaunchConfiguration("roll"),
                "pitch": LaunchConfiguration("pitch"),
                "yaw": LaunchConfiguration("yaw"),
            }.items()
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),

        # BCR bot related nodes
        robot_state_publisher,
        spawn_entity
    ])
