from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    moveit_control = Node(
        package='ros2_manipulation',
        executable='rx200_pick_place',
        parameters=[
            LaunchConfiguration('params_file')
        ]
    )
    
    command_node = Node(
        package='ros2_manipulation',
        executable='pick_place_command'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=['${pwd}/src/ros2_manipulation/params/rx200_params.yaml'],
        description='Path to the ROS2 parameters file to use'
    )

    ld.add_action(params_file)

    ld.add_action(moveit_control)
    ld.add_action(command_node)

    return ld