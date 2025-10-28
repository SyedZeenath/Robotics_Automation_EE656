## Project Details:

- Description: Main assignment demonstrating a ROS2 manipulation (pick & place).
- Objective:
    - Start from a known arm posture.
    - Move to a pick pose, grasp the object.
    - Move to a place pose, release the object.
    - Compute a trajectory composed of straight-line segments between waypoints derived from three reference 

- Consideration: (To Be Updated as and when required)
    - points:
        - arm start pose
        - pick pose
        - place pose


How to run(Make sure you are Inside folder Assignment_1):

Command to run the application:
```bash
ros2 launch robot_launch control_only.launch.py params_file:=$(pwd)/src/ros2_manipulation/params/rx200_params.yaml
```
Command to run the Rviz('fake' is used here to see the simulation):
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=fake
```

Temporary details:
- 'rx200_params.yaml' file is used to play around and understand the points. Location: 'Assignment_1/src/       ros2_manipulation/params/rx200_params.yaml'
- All you need to do is update the points and see how the arm moves. You need not build or source everytime for this.
- This will be updated once we decide the waypoints and compute using cartesian method.