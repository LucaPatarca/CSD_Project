# Copyright 2020 Louise Poubel
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

"""Launch Gazebo with a world that has Robot, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_simulation = get_package_share_directory('simulation')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # TODO: expose pose as a launch argument
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'robot',
                    '-x', '5.0',
                    '-z', '1.0',
                    '-Y', '1.57',
                    '-file', os.path.join(pkg_simulation, 'models', 'robot',
                                          'model.sdf')],
                 output='screen')

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robot/laser_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_simulation, 'rviz', 'simulation.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg_simulation, 'worlds', 'track_drive_test.world') +
                         ' -v 2 --gui-config ' +
                         os.path.join(pkg_simulation, 'ign', 'gui.config'), ''],
          description='Ignition Gazebo arguments'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        spawn,
        # bridge,
        # rviz
    ])
