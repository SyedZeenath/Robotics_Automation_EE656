from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    ld = LaunchDescription()
    # ------------------------------
    # Parameters file
    # ------------------------------
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.dirname(__file__),
            '..', '..', 'ros2_perception', 'params', 'rx200_params.yaml'
        ),
        description='Path to the ROS2 parameters file to use'
    )
    ld.add_action(params_file)
    
    # ------------------------------
    # Include Interbotix perception launch
    # ------------------------------
    #ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=rx200
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'launch',
                'xsarm_perception.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': 'rx200',
            'use_pointcloud_tuner_gui': 'true',
            'hardware_type': 'fake',
        }.items()
    )

    ld.add_action(perception_launch)
    
    # ------------------------------
    # Static transform: wrist -> camera
    # ------------------------------
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_tf",
        output="screen",
        arguments=["0.0", "0.0", "0.0",   # x, y, z offset
                   "0.0", "0.0", "0.5",   # roll, pitch, yaw
                   "rx200/wrist_link",          # parent frame
                   "camera_link"]  # child frame
    )
    ld.add_action(camera_static_tf)

    # Include MoveIt2 launch 
    #ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': 'rx200',
            'hardware_type': 'fake',
            'use_moveit_rviz': 'false'
        }.items()
    )
    
    ld.add_action(moveit_launch)
    
    # ------------------------------
    # MoveIt control node
    # ------------------------------
    moveit_control = Node(
        package='ros2_perception',
        executable='rx200_pick_place',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    # ld.add_action(moveit_control)
    # ------------------------------
    # Perception node
    # ------------------------------
    perception_node = Node(
        package='ros2_perception',
        executable='pick_place_perception',
        output='screen'
    )
    ld.add_action(perception_node)


    return ld
