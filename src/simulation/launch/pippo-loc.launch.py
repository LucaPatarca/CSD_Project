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
    
    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # namespace="pippo",
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        name='map_transform',
        output='screen',
        arguments = "--x -1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        )
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        namespace="pippo",
        output='screen',
        parameters=[{
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False,
            "publish_filtered_gps": False,
            "broadcast_utm_transform": False,
            "use_simtime": True,
        }],
        remappings=[
            # ('/pippo/odometry/filtered', '/pippo/odom'),
            # ("/tf", "tf"), ("/tf_static", "tf_static"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        )

    ukf_localization_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='pippo_ukf_node',
        # namespace="pippo",
        output='screen',
        respawn=True,
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[os.path.join(pkg_simulation, 'config/ukf.yaml'), {'use_sim_time': True}],
        remappings=[
            # ('/pippo/odometry/filtered', '/pippo/odom'),
            # ("/tf", "tf"), ("/tf_static", "tf_static")
        ]
        )
    return LaunchDescription([
        map_transform_node,
        # navsat_transform_node,
        ukf_localization_node,
    ])
