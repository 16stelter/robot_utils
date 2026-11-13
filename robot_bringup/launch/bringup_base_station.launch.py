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
  use_udp_bridge = LaunchConfiguration('udp_bridge').perform(context).lower()
  use_openvins = LaunchConfiguration('use_openvins').perform(context).lower()
  use_rviz = LaunchConfiguration('rviz').perform(context).lower()

  udp_bridge_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('udp_bridge'), 'launch', 'udp_bridge.launch.py')),
          launch_arguments={
              'namespace': namespace,
              'config_file': 'base.yaml',
          }.items(),
          condition=IfCondition(use_udp_bridge),
  )

  openvins_cmd = Node(
    package="ov_msckf",
    executable="run_subscribe_msckf",
    condition=IfCondition(use_openvins),
    namespace=namespace,
    output='screen',
    parameters=[
      {"verbosity": "INFO"},
      {"use_stereo": False},
      {"max_cameras": 1},
      {"save_total_state": False},
      {"config_path": os.path.join(get_package_share_directory('robot_bringup'), 'config', 'open_vins_rs', 'camera_a', 'estimator_config_320.yaml')},
    ],
  remappings=[("/tf", "tf_ov"), ("/tf_static", "tf_static_ov")], # i would like to disable this entirely, but openvins has no option to disable tf publishing, so we remap it to a dummy topic
  )

  image_transport_cmd = Node(
    package='image_transport',
    executable='republish',
    name='image_republisher',
    namespace=namespace,
    parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
    remappings=[
      ('in/compressed', '/exomy/camera/color/image_raw/compressed'),
      ('out', '/exomy/camera/color/image_raw')
    ],
    condition=IfCondition(use_udp_bridge),
  )

  rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    namespace=namespace,
    arguments=['-d', os.path.join(get_package_share_directory('exomy_description'), 'config', 'open_vins', 'display_ros2.rviz')],
    condition=IfCondition(use_rviz),
    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
  )

  return [udp_bridge_cmd, openvins_cmd, image_transport_cmd, rviz_cmd]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='exomy', description='Top-level namespace'
    )

    declare_udp_bridge_cmd = DeclareLaunchArgument(
        'udp_bridge', default_value='true', description='Whether to launch the UDP bridge node'
    )

    declare_use_openvins_cmd = DeclareLaunchArgument(
        'use_openvins', default_value='true', description='Whether to launch the OpenVINS node'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false', description='Whether to launch RViz'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_udp_bridge_cmd,
        declare_use_openvins_cmd,
        declare_rviz_cmd,
        OpaqueFunction(function=launch_setup)
    ])