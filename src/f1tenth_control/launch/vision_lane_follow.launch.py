from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_control')
    config_dir = PathJoinSubstitution([FindPackageShare('f1tenth_control'), 'config'])

    lane_detector_config = PathJoinSubstitution([config_dir, 'lane_detector.yaml'])
    cone_detector_config = PathJoinSubstitution([config_dir, 'cone_detector.yaml'])
    construction_zone_mux_config = PathJoinSubstitution([config_dir, 'construction_zone_mux.yaml'])
    pure_pursuit_config = PathJoinSubstitution([config_dir, 'vision_pp.yaml'])
    camera_config = PathJoinSubstitution([config_dir, 'camera.yaml'])
    mocap_config = PathJoinSubstitution([FindPackageShare('motion_capture_tracking'), 'config', 'cfg.yaml'])

    launch_args = [
        DeclareLaunchArgument('car_name', default_value='car1'),
        DeclareLaunchArgument('start_cam', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('start_visualization', default_value='false'),
        DeclareLaunchArgument('start_mocap', default_value='false'),  # Disabled by default for odom-free mode
        DeclareLaunchArgument('camera_image_topic', default_value='/car1/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/car1/camera/color/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/car1/odom'),
    ]

    car_name = LaunchConfiguration('car_name')
    start_cam = LaunchConfiguration('start_cam')
    start_lidar = LaunchConfiguration('start_lidar')
    start_visualization = LaunchConfiguration('start_visualization')
    start_mocap = LaunchConfiguration('start_mocap')

    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    lane_detector_node = Node(
        package='f1tenth_control',
        executable='lane_detector_node',
        name='lane_detector',
        output='screen',
        parameters=[
            lane_detector_config,
            {
                'image_topic': camera_image_topic,
                'camera_info_topic': camera_info_topic,
                'odom_topic': odom_topic,
            },
        ],
        remappings=[
            ('/car1/camera/color/image_raw', '/car1/D435i/color/image_raw'),
            ('/car1/camera/color/camera_info', '/car1/D435i/color/camera_info'),
        ],
    )

    cone_detector_node = Node(
        package='f1tenth_control',
        executable='cone_detector_node',
        name='cone_detector',
        output='screen',
        parameters=[cone_detector_config],
    )

    construction_zone_mux_node = Node(
        package='f1tenth_control',
        executable='construction_zone_mux',
        name='construction_zone_mux',
        output='screen',
        parameters=[construction_zone_mux_config],
    )

    pure_pursuit_node = Node(
        package='f1tenth_control',
        executable='pure_pursuit_control',
        name='pure_pursuit_control',
        output='screen',
        emulate_tty=True,
        parameters=[pure_pursuit_config],
        remappings=[
            ('/odom', [car_name, '/odom'])
        ],
    )

    mocap_node = Node(
        package='motion_capture_tracking',
        executable='3d_pose_motion_capture_node',
        name='motion_capture_tracking',
        output='screen',
        parameters=[mocap_config],
        condition=IfCondition(start_mocap),
    )

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[PathJoinSubstitution([config_dir, 'sensors.yaml'])],
        output='screen',
        condition=IfCondition(start_lidar),
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='D435i',
        namespace=car_name,
        parameters=[camera_config],
        output='screen',
        condition=IfCondition(start_cam),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('f1tenth_control'), '/launch/teleop.launch.py'
        ])
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('f1tenth_control'), '/launch/visualization_launch.py'
        ]),
        condition=IfCondition(start_visualization),
    )

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=[
            '0.08875938504934311',
            '-0.3511563539505005',
            '-0.004507781006395817',
            '0.013775953091681004',
            '-0.005556662101298571',
            '0.9736091494560242',
            '-0.22773799300193787',
            'world',
            'map'
        ],
        output='screen'
    )

    ld = LaunchDescription(launch_args)
    ld.add_action(mocap_node)
    ld.add_action(urg_node)
    ld.add_action(realsense_node)
    ld.add_action(teleop_launch)
    ld.add_action(visualization_launch)
    ld.add_action(lane_detector_node)
    ld.add_action(cone_detector_node)
    ld.add_action(construction_zone_mux_node)
    ld.add_action(pure_pursuit_node)
    ld.add_action(static_transform_node)

    return ld
