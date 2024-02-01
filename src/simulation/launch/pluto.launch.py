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

    pkg_simulation = get_package_share_directory('simulation')
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="pluto_state_publisher",
        namespace="pluto",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"robot_description": Command(["xacro ", os.path.join(pkg_simulation, "urdf/pluto.urdf")])}
        ],
    )
    
    spawn = Node( 
        package='ros_gz_sim',
        executable='create',
        # namespace="pluto",
        arguments=[ 
            '-name', 'pluto',
            '-topic', '/pluto/robot_description',
            '-z', '0.2',
            '-x', '2',
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
            "-c", "pluto/controller_manager",
            "pluto_joint_state_broadcaster",
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
            "-c", "pluto/controller_manager",
            "pluto_diff_drive_controller",
        ],
        shell=False,
        output="screen",
    )

    return LaunchDescription([
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
    ])