# Copyright 2024 Intelligent Robotics Lab
#
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_scan_topic',
            default_value='/scan',
            description='Input LaserScan topic'
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
            description='Robot frame for transformation'
        ),
        DeclareLaunchArgument(
            'obstacle_frame',
            default_value='nearest_obstacle',
            description='Frame ID for the obstacle TF'
        ),
        Node(
            package='laser',
            executable='nearest_obstacle',
            name='nearest_obstacle_node',
            output='screen',
            parameters=[{
                'robot_frame': LaunchConfiguration('robot_frame'),
                'obstacle_frame': LaunchConfiguration('obstacle_frame'),
                'range_min_threshold': 0.0,
                'range_max_threshold': 10.0,
                'enable_debug_output': True
            }],
            remappings=[
                ('input_scan', LaunchConfiguration('input_scan_topic'))
            ]
        )
    ])