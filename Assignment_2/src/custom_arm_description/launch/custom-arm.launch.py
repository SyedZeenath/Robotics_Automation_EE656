from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'custom_arm_description', 'share', 'custom_arm_description'
    )
    urdf_path = os.path.join(pkg_share, 'urdf', 'custom_arm.urdf.xacro')
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' robot_name:=rx200',
            ' use_gripper:=true',
            ' base_link_frame:=base_link'
        ]),
        value_type=str
    )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',

        #     output='screen',
        # )
    ])
