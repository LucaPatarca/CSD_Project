"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import Condition, LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_simulation = get_package_share_directory('simulation')
    gz_models_path = os.path.join(pkg_simulation, "models")

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="pippo",
        name="rviz2",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
        arguments=["-d", os.path.join(pkg_simulation, "rviz/simulation.rviz")],
    )
    
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/pippo/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/pippo/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            '/pippo/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )
    
    pippo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_simulation, 'launch/pippo.launch.py'))
    )
    
    pluto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_simulation, 'launch/pluto.launch.py'))
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=gz_models_path,
        ),
        SetEnvironmentVariable(
            name="IGN_GAZEBO_MODEL_PATH",
            value=gz_models_path,
        ),
        DeclareLaunchArgument(
          'gz_args',
          default_value=[os.path.join(pkg_simulation, 'worlds', 'empty.sdf') + ' -r'],
          description='Ignition Gazebo arguments'),
        gazebo,
        rviz_node,
        bridge,
        pippo,
        pluto,
    ])