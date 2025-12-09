## Project Details:

- ### Description: 

  Assignment demonstrating picking of three colored blocks and stacking one over the other at a pre-defined position. The pick order is passed as parameters. The idea here is, the arm reads the pick order of the blocks, it then detects the blocks through the camera connected. Based on the order, the arm picks each block and places them one over another in a predefined position. The pickorder is passed through the params file, which can be modified to any order of choice.

- ### Sequence followed:

  There are two nodes running: one for perception and one for pick and place. The perception node calls the /pc_filter/get_cluster_positions service to get the color and position of each block. After that, the block positions are transformed into the correct reference frame, cleaned up, and published on the /detected_blocks topic.

  The pick-and-place node subscribes to this topic so it can read the block positions and colors. Using the given pick order, it then controls the arm to carry out the pick-and-place steps. The motion sequence is the same one used in Assignment 1.

  Sequence(Brief):

  The robotic arm begins by moving close to the spot where it needs to pick up the block, pausing just before reaching the *before_pick* in the code. It then glides carefully into the pick position. Once there, the gripper closes, indicating that the pick is complete.

  With the block held, the arm lifts it smoothly to the *above_pick* position, making sure it’s safely clear of any obstacles. It then moves to the *above_place* point, positioning itself above where the block will be set down. From there, the arm lowers to the *place_point* and gently opens the gripper, releasing the block into place. the *place_point* here is the *stack_pos* and the z-axis of it is dynamically calculated based on the height of blocks.

- ### Points taken into consideration:

  - block_height - a constant value which acts as a height of the block, which is used to calculate the stacking position over already placed block.
  - lift_height - a constant value is given to lift the block above the ground. This value is added to the z-value of a point.
  - stack_pos - a constant value indicating the position where the blocks need to be stacked. It is currently pre-defined in the code and not provided through params.

## How to run(Make sure you are in folder Assignment_2):
 Build and source
 ```bash
    colcon build
    source install/setup.bash
 ```
Command to run the application:
```bash
 ros2 launch robot_launch control_only.launch.py params_file:=$(pwd)/src/ros2_perception/params/rx200_params.yaml
```
Note: params_file is the location where the params file is located. Either give a full path or go to the 'Assignment_2' folder and copy the command as shown above

Command to run the Rviz('fake' is used here to see the simulation, update to actual or gazebo if needed):
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual
```
(To be edited later: temp perception command put up here)
```bash
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=rx200
```

## Folder Structure:
- Assignment-2:
    - src/
      - ros2_perception/ros2_perception
        - Description: Assignment demonstrating the pick & stack three colored blocks - has two nodes, one for detecting the blocks publishing their color&positions(perception Node) and other one where the movement of arm updates(client Node).
        - rx200_pick_place_perception.py(perception Node):
          - Description: This is a node that publishes the color and position of the blocks to the Client Node
        - rx200_pick_place_client.py(client Node):
          - Description: Apart from the code done in the lab(initializing and defining constraints), this node subscribes to the perception node. A series of sequence as done in the assignment_1 has been used here to move the arms.
      - ros2_perception/params/rx200_params.yaml
        - The file to modify the stack position and other parameters passed into the application.
      - robot_launch/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.

## Params file

This file takes one parameter.

- pick_order: An array containing the order of colors to be picked and stacked.

## Team Members
- Zeenath Ara Syed
- Deepika Nayudu
- Katie Brugha