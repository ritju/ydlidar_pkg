#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def get_environment_value(var, env, default):
    try:
        if env in os.environ:
            var[0] = os.environ.get(env)
            print(f'get {env} value: {var[0]} from docker_compose.yaml file')
        else:
            var[0] = default
            print(f"Using default {env} value: {default}.")
    except Exception as e:
        print(f'exception: {str(e)}')
        print(f"Please input {env} in docker_compose.yaml")
        var[0] = default


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file_ydlidar')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument('params_file_ydlidar',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
    # log level control
    log_level_l = LaunchConfiguration('log_level_l', default='info')
    log_level_r = LaunchConfiguration('log_level_r', default='info')
    
    ydlidar_l_port = ["/dev/ydlidar"]
    ydlidar_r_port = ["/dev/ydlidar"]

    get_environment_value(ydlidar_l_port, "LIDAR_L_PORT", "/dev/ydlidar_l")
    get_environment_value(ydlidar_r_port, "LIDAR_R_PORT", "/dev/ydlidar_r")

    # left laser
    angle_min1_l = [0]
    angle_max1_l = [90]
    angle_min2_l = [270]
    angle_max2_l = [360]

    get_environment_value(angle_min1_l, "ANGLE_MULMIN_1_L", 0)
    get_environment_value(angle_max1_l, "ANGLE_MULMAX_1_L", 90)
    get_environment_value(angle_min2_l, "ANGLE_MULMIN_2_L", 270)
    get_environment_value(angle_max2_l, "ANGLE_MULMAX_2_L", 360)

    # right laser
    angle_min1_r = [0]
    angle_max1_r = [90]
    angle_min2_r = [270]
    angle_max2_r = [360]

    get_environment_value(angle_min1_r, "ANGLE_MULMIN_1_R", 0)
    get_environment_value(angle_max1_r, "ANGLE_MULMAX_1_R", 90)
    get_environment_value(angle_min2_r, "ANGLE_MULMIN_2_R", 270)
    get_environment_value(angle_max2_r, "ANGLE_MULMAX_2_R", 360)
    
    params_extra_l = {
        "port": ydlidar_l_port[0], 
        "frame_id": "ydscan_l",
        "angle_min1": int(angle_min1_l[0]),
        "angle_max1": int(angle_max1_l[0]),
        "angle_min2": int(angle_min2_l[0]),
        "angle_max2": int(angle_max2_l[0]),
    }
    params_extra_r = {
        "port": ydlidar_l_port[0], 
        "frame_id": "ydscan_r",
        "angle_min1": int(angle_min1_r[0]),
        "angle_max1": int(angle_max1_r[0]),
        "angle_min2": int(angle_min2_r[0]),
        "angle_max2": int(angle_max2_r[0]),
    }

    driver_node_l = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_l',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file, params_extra_l],
                                namespace='/',
                                remappings=[('scan', 'ydscan_l')],
                                arguments=['--ros-args', '--log-level', ['ydlidar_l:=', log_level_l]],
                                respawn=True,
                                )
    
    driver_node_r = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_r',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file, params_extra_r],
                                namespace='/',
                                remappings=[('scan', 'ydscan_r')],
                                arguments=['--ros-args', '--log-level', ['ydlidar_l:=', log_level_r]],
                                respawn=True,
                                )
    
    tf2_node_l = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser_l',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','ydscan_l'],
                    )
    tf2_node_r = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser_r',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','ydscan_r'],
                    )

    return LaunchDescription([
        params_declare,
        driver_node_l,
        driver_node_r,
        # tf2_node_l,
    ])
