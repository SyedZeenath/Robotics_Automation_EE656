from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

import os

def generate_launch_description():

    ld = LaunchDescription()
    
    # ------------------------------
    # Include MoveIt launch
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
            'hardware_type': 'fake',
            'use_moveit_rviz': 'true'
        }.items()
    )
    
    # ------------------------------
    # Include Perception launch
    # ------------------------------
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'launch',
                'xsarm_perception.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': 'rx200',
            'hardware_type': 'fake',
            'use_pointcloud_tuner_gui': 'true',
            'use_rviz': 'true'
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
            "0.09", "0.0", "0.10",          # x, y, z translation
            "0.92", "0.0", "0.0", "0.38",  # quaternion x, y, z, w
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
    
    wait_for_rx200_node = RegisterEventHandler(
        OnProcessStart(
            target_action=rx200_node,
            on_start=[perception_node]
        )
    )

    ld.add_action(moveit_launch)
    ld.add_action(perception_launch),
    ld.add_action(camera_static_tf)
    ld.add_action(rx200_node)
    # ld.add_action(wait_for_rx200_node)

    return ld
