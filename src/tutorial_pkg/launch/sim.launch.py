from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    tutorial_pkg = FindPackageShare('tutorial_pkg')
    gazebo_pkg = FindPackageShare('rosbot_xl_gazebo')
    map_merge_pkg = FindPackageShare('multirobot_map_merge')

    explore_launch_path = PathJoinSubstitution(
        [tutorial_pkg, 'launch', 'explore.launch.py']
    )
    gazebo_launch_path = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'simulation.launch.py']
    )

    gazebo_spawn_path = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'spawn.launch.py']
    )

    map_merge_launch_path = PathJoinSubstitution(
        [map_merge_pkg, 'launch', 'map_merge.launch.py']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    world_file = PathJoinSubstitution([tutorial_pkg, "worlds", "house.world"])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments={
            'namespace': 'robot1',
            'camera_model': 'None',
            'x': '1.0',
            'y': '3.0',
            'world': world_file,
        }.items(),
    )

    gazebo_launch_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_spawn_path]),
        launch_arguments={
            'namespace': 'robot2',
            'x': '1.0',
            'y': '-4.0',
            'use_sim_time': 'True',
            'camera_model': 'None',
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="robot1",
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([tutorial_pkg, "rviz", "explore.rviz"])],
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': 'robot1',
        }.items(),
    )

    rviz_node_robot2 = Node(
        package="rviz2",
        executable="rviz2",
        namespace="robot2",
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([tutorial_pkg, "rviz", "explore.rviz"])],
    )

    explore_launch_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': 'robot2',
        }.items(),
    )

    battery_robot1 = Node(
        package="tutorial_pkg",
        executable="battery_state_pub.py",
        namespace="robot1",
        name="robot1_battery",
    )

    battery_controller_robot1 = Node(
        package="tutorial_pkg",
        executable="controller.py",
        namespace="robot1",
        name="robot1_battery_controller",
    )

    battery_robot2 = Node(
        package="tutorial_pkg",
        executable="battery_state_pub.py",
        namespace="robot2",
        name="robot2_battery",
    )

    battery_controller_robot2 = Node(
        package="tutorial_pkg",
        executable="controller.py",
        namespace="robot2",
        name="robot2_battery_controller",
    )

    map_merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([map_merge_launch_path]),
        launch_arguments={
            # 'known_init_poses': 'False',
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            gazebo_launch,
            gazebo_launch_robot2,
            rviz_node,
            explore_launch,
            rviz_node_robot2,
            explore_launch_robot2,
            battery_robot1,
            battery_controller_robot1,
            battery_robot2,
            battery_controller_robot2,
            # map_merge_launch,
        ]
    )
