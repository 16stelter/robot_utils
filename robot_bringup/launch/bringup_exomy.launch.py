import os, yaml
from tempfile import NamedTemporaryFile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    ns_prefix = f"{namespace}/" if namespace else ""
    use_imu = LaunchConfiguration('imu').perform(context).lower()
    use_arducam = LaunchConfiguration('arducam').perform(context).lower()
    use_realsense = LaunchConfiguration('realsense').perform(context).lower()
    use_udp_bridge = LaunchConfiguration('udp_bridge').perform(context).lower()
    robot_lowlevel_dir = get_package_share_directory('exomy')
    robot_desc_dir = get_package_share_directory('exomy_description')

    robot_lowlevel_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_lowlevel_dir, 'exomy.launch.py')),
            launch_arguments={'namespace': namespace}.items())

    arducam_node_cmd = Node(
        package = 'arducam_rclpy_tof_pointcloud',
        executable = 'tof_pointcloud',
        name = 'tof_pointcloud',
        namespace = namespace,
        output = 'screen',
        parameters=[{'namespace': ''},
                    {'margin_x': 20},
                    {'margin_y': 10}],
        condition=IfCondition(use_arducam),
    )

    imu_params = os.path.join(get_package_share_directory('imu_wrapper'), 'imu.yaml')
    configured_imu_params = prepend_namespace_to_yaml(imu_params, namespace)
    imu_node_cmd = Node(
        package = 'imu_wrapper',
        executable = 'imu_wrapper',
        name = 'imu_wrapper',
        namespace = namespace,
        output = 'screen',
        parameters=[configured_imu_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        condition=IfCondition(use_imu),
    )

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            launch_arguments={
                'camera_namespace': namespace,
                'initial_reset': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'rgb_camera.color_profile': '320x240x60',
                'enable_depth': 'false'
            }.items(),
            condition=IfCondition(use_realsense),
    )

    udp_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('udp_bridge'), 'launch', 'udp_bridge.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'config_file': 'exomy.yaml',
            }.items(),
            condition=IfCondition(use_udp_bridge),
    )

    robot_desc_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(robot_desc_dir, 'launch', 'robot.launch.py')),
        launch_arguments={'namespace': namespace, 'prefix': ns_prefix}.items())

    odom_cmd = Node(
        package='exomy',
        executable='odometry_node',
        name='odometry_node',
        namespace=namespace,
        output='screen',
        parameters=[{'namespace': namespace}],
    )

    control_cmd = Node(
        package='control',
        executable='purepursuit',
        name='purepursuit',
        namespace=namespace,
        output='screen',
        parameters=[{'namespace': namespace}],
    )

    return [robot_desc_cmd, robot_lowlevel_cmd, arducam_node_cmd, imu_node_cmd, realsense_cmd, udp_bridge_cmd, odom_cmd, control_cmd]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_imu_cmd = DeclareLaunchArgument(
        'imu', default_value='false', description='Whether to launch the internal IMU node'
    )

    declare_arducam_cmd = DeclareLaunchArgument(
        'arducam', default_value='false', description='Whether to launch the Arducam ToF camera node'
    )

    declare_realsense_cmd = DeclareLaunchArgument(
        'realsense', default_value='true', description='Whether to launch the Realsense camera node'
    )

    declare_udp_bridge_cmd = DeclareLaunchArgument(
        'udp_bridge', default_value='true', description='Whether to launch the UDP bridge node'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_imu_cmd,
        declare_arducam_cmd,
        declare_realsense_cmd,
        declare_udp_bridge_cmd,
        OpaqueFunction(function=launch_setup)
    ])

def prepend_namespace_to_yaml(input_file, namespace):
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    namespaced_data = {}
    for node_name, node_config in data.items():
        if namespace:
            if 'ros__parameters' in node_config and 'target_frame' in node_config['ros__parameters']:
                node_config['ros__parameters']['target_frame'] = f'{namespace}/{node_config["ros__parameters"]["target_frame"]}'
            namespaced_key = f'/{namespace}/{node_name}'
        else:
            namespaced_key = f'/{node_name}'

        namespaced_data[namespaced_key] = node_config

    tmp_file = NamedTemporaryFile(delete=False, mode='w', suffix='.yaml')
    yaml.dump(namespaced_data, tmp_file)
    tmp_file.close()

    return tmp_file.name
