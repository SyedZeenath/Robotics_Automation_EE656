from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # TODO: later
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="camera_static_tf",
    #     arguments=["0.30", "0.15", "0.50", "0", "0", "1.57", "base_link", "camera_link"]
    # )
    
    moveit_control = Node(
        package='ros2_perception',
        executable='rx200_pick_place',
        parameters=[
            LaunchConfiguration('params_file')
        ]
    )
    
    command_node = Node(
        package='ros2_perception',
        executable='pick_place_perception'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=['${pwd}/src/ros2_perception/params/rx200_params.yaml'],
        description='Path to the ROS2 parameters file to use'
    )
    
        # Include Interbotix perception launch file
    interbotix_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_launch'),
                'launch',
                'perception.launch.py'
            )
        ),
    )
    
    ld.add_action(params_file)

    ld.add_action(moveit_control)
    ld.add_action(command_node)
    ld.add_action(interbotix_perception_launch)

    return ld