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
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="pippo",
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    
    spawn = Node( 
        package='ros_gz_sim',
        executable='create',
        arguments=[ 
            '-name', 'pippo',
            '-topic', '/pippo/robot_description',
            '-z', '0.2',
            '-x', '-2',
            '-y', '0',
        ],
        output='screen',
    )
    
    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state", "active",
            "-c", "pippo/controller_manager",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )
    
    load_diff_drive_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state", "active",
            "-c", "pippo/controller_manager",
            "diff_drive_base_controller",
        ],
        shell=False,
        output="screen",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace='pippo',
        name="rviz2",
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
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
          default_value=[os.path.join(pkg_simulation, 'worlds', 'empty.sdf') + ' -r'],
          description='Ignition Gazebo arguments'),
        DeclareLaunchArgument(
            name="model",
            default_value=os.path.join(pkg_simulation, "urdf/sam_bot.urdf"),
            description="Absolute path to robot urdf file",),
        gazebo,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        rviz_node,
        bridge,
        # spawn2,
    ])