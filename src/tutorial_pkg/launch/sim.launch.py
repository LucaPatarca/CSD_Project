from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    tutorial_pkg = FindPackageShare('tutorial_pkg')
    gazebo_pkg = FindPackageShare('rosbot_xl_gazebo')

    explore_launch_path = PathJoinSubstitution(
        [tutorial_pkg, 'launch', 'explore.launch.py']
    )
    gazebo_launch_path = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'simulation.launch.py']
    )

    gazebo_spawn_path = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'spawn.launch.py']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments={
            'namespace': 'pippo',
            'camera_model': 'None',
        }.items(),
    )

    gazebo_launch_pluto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_spawn_path]),
        launch_arguments={
            'namespace': 'pluto',
            'x': '-1.0',
            'y': '2.0',
            'use_sim_time': 'True',
            'camera_model': 'None',
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="pippo",
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([tutorial_pkg, "rviz", "explore.rviz"])],
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': 'pippo',
        }.items(),
    )

    rviz_node_pluto = Node(
        package="rviz2",
        executable="rviz2",
        namespace="pluto",
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([tutorial_pkg, "rviz", "explore.rviz"])],
    )

    explore_launch_pluto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': 'pluto',
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            gazebo_launch,
            gazebo_launch_pluto,
            rviz_node,
            explore_launch,
            rviz_node_pluto,
            explore_launch_pluto,
        ]
    )
