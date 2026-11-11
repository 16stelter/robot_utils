import os, yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tempfile import NamedTemporaryFile

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower()

    openvins_cmd = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace=namespace,
        output='screen',
        parameters=[
        {"verbosity": "INFO"},
        {"use_stereo": False},
        {"max_cameras": 1},
        {"save_total_state": False},
        {"config_path": os.path.join(get_package_share_directory('robot_bringup'), 'config', 'open_vins_rs', 'estimator_config.yaml')},
        ],
    remappings=[("/tf", "tf_ov"), ("/tf_static", "tf_static_ov")], # i would like to disable this entirely, but openvins has no option to disable tf publishing, so we remap it to a dummy topic
    )

    odom_cmd = Node(
        package='exomy',
        executable='odometry_node',
        name='odometry_node',
        namespace=namespace,
        output='screen',
        parameters=[{'namespace': namespace,
                     'base_frame': 'camera_link'}],
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['-d', os.path.join(get_package_share_directory('robot_bringup'), 'config', 'open_vins_rs', 'display_ros2.rviz')],
        condition=IfCondition(use_rviz),
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return [openvins_cmd, odom_cmd, rviz_cmd]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to launch RViz'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_rviz_cmd,
        OpaqueFunction(function=launch_setup)
    ])