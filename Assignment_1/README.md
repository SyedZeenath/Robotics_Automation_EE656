## Project Details:

- Description: Main assignment demonstrating a ROS2 manipulation (pick & place).
- Objective:
    - Start from a known arm posture.
    - Move to a pick pose, grasp the object.
    - Move to a place pose, release the object.
    - Compute a trajectory composed of straight-line segments between waypoints derived from three reference 

- Consideration:
    - points:
        - pick pose - Can be passed through the rx200_params.yaml file 
        - place pose - Can be passed through the rx200_params.yaml file 
        - Two intermediate points are defined - one for picking *above_pick* and one for placing *above_place*. These points are calculated by adding a fixed threshold value to the y-coordinate of both the pick and place positions. This added value represents the lift height, showing how much the arm raises the object, since the z-axis is not used in this case.
        - a constant z value is defined to avoid ground collision

- Sequence followed:

    The robotic arm begins by moving close to the spot where it needs to pick up the object, pausing just before reaching it the *before_pick* in the code. It then glides carefully into the pick position. Once there, the gripper closes, indicating that the pick is complete.

    With the object held, the arm lifts it smoothly to the *above_pick* position, making sure it’s safely clear of any obstacles. It then travels across to the *above_place* point, positioning itself above where the object will be set down. From there, the arm lowers to the *place_point* and gently opens the gripper, releasing the object into place.

    Finally, the arm rises back to the *above_place* position, marking the end of the pick-and-place sequence.

## How to run(Make sure you are Inside folder Assignment_1):
 Build and source
 ```bash
    colcon build
    source install/setup.bash
 ```
Command to run the application:
```bash
ros2 launch robot_launch control_only.launch.py params_file:=$(pwd)/src/ros2_manipulation/params/rx200_params.yaml
```
Command to run the Rviz('fake' is used here to see the simulation, update to actual or gazebo if needed):
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=fake
```

## Folder Structure:
- Assignment-1:
    - src/
      - ros2_manipulation/ 
        - Description: Assignment demonstrating a ROS2 manipulation (pick & place) - has two nodes, one for publishing the sequence along with timer(Command Node) and one where the movement of arm updates(client Node).
            - rx200_pick_place_command.py(Command Node):
              - Description: This is a node that publishes the state the arm has to move (commands) every 5 seconds to the Client Node
            - rx200_pick_place_client.py(client Node):
              - Description: Apart from the code done in the lab(initializing and defining constraints), this node subscribes to the command node. A series of sequence as mentioned above has been numbered from 1-9 as *state* of the arm. The client node reads what state it has published and makes the arm move based on the published state. 
      - robot_launch/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.