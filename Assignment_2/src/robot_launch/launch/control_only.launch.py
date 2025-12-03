from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    #TODO: Need to fix the loading of urdf from external file
    # Path to your external URDF/XACRO
    external_urdf = '/home/zeenath/Robotics_Automation_EE656/Assignment_2/src/ros2_perception/robot_urdf/urdf/rx200.urdf.xacro'
    
    # Path to Interbotix perception and moveit launch files
    perception_pkg_share = get_package_share_directory('interbotix_xsarm_perception')
    moveit_pkg_share = get_package_share_directory('interbotix_xsarm_moveit')
    robot_urdf_pkg_path = get_package_share_directory('interbotix_xsarm_descriptions')
    
    description_launch_path = os.path.join(robot_urdf_pkg_path, 'launch', 'xsarm_description.launch.py')

    # Include your robot description launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
    )
    
    # ------------------------------
    # Load Urdf file
    # ------------------------------
    # urdf_file = os.path.join(
    #     get_package_share_directory('robot_urdf'),
    #     'urdf',
    #     'rx200.urdf.xacro'
    # )

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
    # Static transform: wrist -> camera
    # ------------------------------
    # camera_static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="camera_static_tf",
    #     output="screen",
    #     arguments=["0.0", "0.0", "0.1",   # x, y, z offset
    #                "0.0", "0.0", "0.0",   # roll, pitch, yaw
    #                "rx200/wrist_link",          # parent frame
    #                "camera_color_optical_frame"]  # child frame
    # )
    # ld.add_action(camera_static_tf)

    # ------------------------------
    # Include Interbotix perception launch
    # ------------------------------
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg_share, 'launch', 'xsarm_perception.launch.py')
        ),
        launch_arguments={
            'robot_model': 'rx200',
            'use_pointcloud_tuner': 'true',
            'use_perception': 'true'
        }.items()
    )

    # ld.add_action(perception_launch)
    
    # Include MoveIt2 launch 
    #ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_share, 'launch', 'xsarm_moveit.launch.py')
        ),
        launch_arguments={
            'robot_model': 'rx200',
            'hardware_type': 'fake',
            'use_rviz': 'false'
        }.items()
    )
    
    ld.add_action(moveit_launch)

    # ------------------------------
    # Perception node
    # ------------------------------
    perception_node = Node(
        package='ros2_perception',
        executable='pick_place_perception',
        output='screen'
    )
    ld.add_action(perception_node)
    
    # ------------------------------
    # MoveIt control node
    # ------------------------------
    moveit_control = Node(
        package='ros2_perception',
        executable='rx200_pick_place',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    ld.add_action(moveit_control)

    return ld
