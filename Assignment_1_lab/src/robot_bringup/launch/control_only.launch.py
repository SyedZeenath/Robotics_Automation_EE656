from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
    ld = LaunchDescription()
 
    dc = DeclareLaunchArgument(
        'default_gr_state',
        default_value='true',
        description='What should the state of the gripper be'
    )

    moveit_control = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        parameters=[{
            'start_state_gripper': LaunchConfiguration('default_gr_state')
        }]
    )
 
    ld.add_action(moveit_control)
    ld.add_action(dc)
 
    return ld