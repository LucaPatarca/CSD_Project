from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("tutorial_pkg"), 'config', 'slam.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=slam_params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'), 
            ('/tf_static', 'tf_static'), 
            ('/map', 'map'), 
            ('/map_metadata', 'map_metadata'),
        ],
        name='slam',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg, 
        slam_params_file_arg, 
        declare_namespace_cmd,
        slam_node
    ])
