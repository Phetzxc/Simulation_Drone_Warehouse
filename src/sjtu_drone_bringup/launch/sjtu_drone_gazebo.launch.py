#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = DeclareLaunchArgument(
        "use_gui", default_value="true", choices=["true", "false"],
        description="Whether to execute gzclient"
    )
    
    # รับค่า spawn pose ผ่าน launch argument
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Initial X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Initial Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.0", description="Initial Z position")
    roll_arg = DeclareLaunchArgument("roll", default_value="0.0", description="Initial roll angle (rad)")
    pitch_arg = DeclareLaunchArgument("pitch", default_value="0.0", description="Initial pitch angle (rad)")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0", description="Initial yaw angle (rad)")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    xacro_file_name = "sjtu_drone.urdf.xacro"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )   
    
    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()
    # get ns from yaml
    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]
    print("namespace: ", model_ns)

    world_file_default = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "small_warehouse.world"
    )

    world_file = LaunchConfiguration('world', default=world_file_default)

    world = DeclareLaunchArgument(
        name='world',
        default_value=world_file_default,
        description='Full path to world file to load'
    )

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    # เตรียม argument สำหรับ spawn_drone node
    def to_str(context, launch_config):
        return launch_config.perform(context)

    def spawn_args(context):
        # ดึงค่าจาก context
        x_val = to_str(context, x)
        y_val = to_str(context, y)
        z_val = to_str(context, z)
        roll_val = to_str(context, roll)
        pitch_val = to_str(context, pitch)
        yaw_val = to_str(context, yaw)
        # ส่ง arguments เป็น list ของ string ตามลำดับ
        return [robot_desc, model_ns, x_val, y_val, z_val, roll_val, pitch_val, yaw_val]

    return LaunchDescription([
        use_gui,
        world,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "frame_prefix": model_ns + "/"}],
            arguments=[robot_desc]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': "true",
                'extra_gazebo_args': 'verbose'
            }.items()
        ),

        OpaqueFunction(function=launch_gzclient),

        OpaqueFunction(
            function=lambda context, *args, **kwargs: [
                Node(
                    package="sjtu_drone_bringup",
                    executable="spawn_drone",
                    arguments=spawn_args(context),
                    output="screen"
                )
            ]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),
    ])
