"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)


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
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="pippo",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    
    spawn = Node( 
        package='ros_gz_sim',
        executable='create',
        arguments=[ 
            '-name', 'sam_bot',
            '-topic', '/pippo/robot_description',
            '-z', '0.2',
            '-x', '-2',
            '-y', '0',
        ],
        output='screen',
    )
    
    # spawn2 = Node( 
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[ 
    #         '-name', 'sam_bot2',
    #         '-topic', 'robot_description',
    #         '-z', '0.2',
    #         '-x', '2',
    #         '-y', '0',
    #     ],
    #     output='screen',
    # )

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
          default_value=[os.path.join(pkg_simulation, 'worlds', 'empty.sdf') + ' -v 2'],
          description='Ignition Gazebo arguments'),
        DeclareLaunchArgument(
            name="model",
            default_value=os.path.join(pkg_simulation, "urdf/sam_bot.urdf"),
            description="Absolute path to robot urdf file",),
        gazebo,
        robot_state_publisher,
        spawn,
        # spawn2,
    ])