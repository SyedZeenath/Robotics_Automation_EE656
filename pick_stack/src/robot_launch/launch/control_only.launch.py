from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)

import os

def generate_launch_description():

    ld = LaunchDescription()
    
    # ------------------------------
    # Interbotix MoveIt launch
    # ------------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': 'rx200',
            'hardware_type': 'actual',
            'rviz_config_file': PathJoinSubstitution([
                FindPackageShare('ros2_perception'),
                'config',
                'rx200_pick_place.rviz'
            ]),
        }.items()
    )

    # ------------------------------
    # Static transform: wrist -> camera
    # ------------------------------
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        output='screen',
        arguments=[
            "0.16", "0.05", "0.1",          # x, y, z translation
            "0.0", "0.85", "0.0",  # quaternion x, y, z, w
            "rx200/wrist_link",              # parent frame
            "camera_link"                    # child frame
        ]
    )
    
    # ------------------------------
    # Pick-and-place Node
    # ------------------------------
    rx200_node = Node(
        package='ros2_perception',
        executable='rx200_pick_place',
        name='rx200_pick_place',
        output='screen'
    )
    
    # ------------------------------
    # Perception node
    # ------------------------------
    perception_node = Node(
        package='ros2_perception',
        executable='pick_place_perception',
        name='pick_place_perception',
        output='screen'
    )

    # ------------------------------
    # Interbotix Perception
    # ------------------------------

    pointcloud_enable_launch_arg = LaunchConfiguration('rs_camera_pointcloud_enable')
    rbg_camera_profile_launch_arg = LaunchConfiguration('rs_camera_rbg_camera_profile')
    depth_module_profile_launch_arg = LaunchConfiguration('rs_camera_depth_module_profile')
    logging_level_launch_arg = LaunchConfiguration('rs_camera_logging_level')
    output_location_launch_arg = LaunchConfiguration('rs_camera_output_location')
    initial_reset_launch_arg = LaunchConfiguration('rs_camera_initial_reset')

    filter_ns_launch_arg = LaunchConfiguration('filter_ns')
    filter_params_launch_arg = LaunchConfiguration('filter_params')
    use_pointcloud_tuner_gui_launch_arg = LaunchConfiguration('use_pointcloud_tuner_gui')
    enable_pipeline_launch_arg = LaunchConfiguration('enable_pipeline')
    cloud_topic_launch_arg = LaunchConfiguration('cloud_topic')


    rs_camera_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'rgb_camera.profile': rbg_camera_profile_launch_arg,
            'depth_module.profile': depth_module_profile_launch_arg,
            'pointcloud.enable': pointcloud_enable_launch_arg,
            'initial_reset': initial_reset_launch_arg,
            'log_level': logging_level_launch_arg,
            'output': output_location_launch_arg,
        }.items()
    )

    pc_filter_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'pc_filter.launch.py',
            ])
        ]),
        launch_arguments={
            'filter_ns': filter_ns_launch_arg,
            'filter_params': filter_params_launch_arg,
            'enable_pipeline': enable_pipeline_launch_arg,
            'cloud_topic': cloud_topic_launch_arg,
            'use_pointcloud_tuner_gui': use_pointcloud_tuner_gui_launch_arg,
        }.items(),
    )

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'load_configs',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'a boolean that specifies whether or not the initial register values (under the '
                "'motors' heading) in a Motor Config file should be written to the motors; as the "
                "values being written are stored in each motor's EEPROM (which means the values "
                'are retained even after a power cycle), this can be set to false after the first '
                'time using the robot. Setting to false also shortens the node startup time by a '
                'few seconds and preserves the life of the EEPROM.'
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_pointcloud_enable',
            default_value='true',
            choices=('true', 'false'),
            description="enables the RealSense camera's pointcloud.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_rbg_camera_profile',
            default_value='640x480x30',
            description='profile for the rbg camera image stream, in `<width>x<height>x<fps>`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_depth_module_profile',
            default_value='640x480x30',
            description='profile for the depth module stream, in `<width>x<height>x<fps>`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_logging_level',
            default_value='info',
            choices=('debug', 'info', 'warn', 'error', 'fatal'),
            description='set the logging level for the realsense2_camera launch include.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_output_location',
            default_value='screen',
            choices=('screen', 'log'),
            description='set the logging location for the realsense2_camera launch include.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_initial_reset',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'On occasions the RealSense camera is not closed properly and due to firmware '
                'issues needs to reset. If set to `true`, the device will reset prior to usage.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_ns',
            default_value='pc_filter',
            description='namespace where the pointcloud related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'config',
                'filter_params.yaml'
            ]),
            description=(
                'file location of the parameters used to tune the perception pipeline filters.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui',
            default_value='true',
            choices=('true', 'false'),
            description='whether to show a GUI that a user can use to tune filter parameters.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_pipeline',
            default_value=LaunchConfiguration('use_pointcloud_tuner_gui'),
            choices=('true', 'false'),
            description=(
                'whether to enable the perception pipeline filters to run continuously; to save '
                'computer processing power, this should be set to `false` unless you are actively '
                'trying to tune the filter parameters; if `false`, the pipeline will only run if '
                'the `get_cluster_positions` ROS service is called.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/camera/camera/depth/color/points',
            description='the absolute ROS topic name to subscribe to raw pointcloud data.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_color_optical_frame',
            description='the camera frame in which the AprilTag will be detected.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_color_topic',
            default_value='/camera/camera/color/image_raw',
            description='the absolute ROS topic name to subscribe to color images.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
            description='the absolute ROS topic name to subscribe to the camera color info.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'load_transforms',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'whether or not the static_trans_pub node should publish any poses stored in the '
                'static_transforms.yaml file at startup; this should only be set to `false` if a '
                'TF chain already exists connecting the camera and arm base_link frame (typically '
                'defined in a URDF), and you would rather use that TF chain as opposed to the one '
                'specified in the static_transforms.yaml file.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'transform_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'config',
                'static_transforms.yaml'
            ]),
            description=(
                'filepath to the static_transforms.yaml file used by the static_trans_pub node; if'
                ' the file does not exist yet, this is where you would like the file to be '
                'generated.'
            ),
        )
    )
    


    for decl_arg in declared_arguments:
        ld.add_action(decl_arg)

    ld.add_action(rs_camera_launch_include)
    ld.add_action(pc_filter_launch_include)
    ld.add_action(moveit_launch)
    ld.add_action(camera_static_tf)
    ld.add_action(rx200_node)
    ld.add_action(perception_node)

    return ld
