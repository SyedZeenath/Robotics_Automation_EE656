## Project Details:

### A Modular Robotic System for Colour-Based Laboratory Sample Sorting

- ### Description:

  Code demonstrating picking of three colored blood samples and placing them at a pre-defined position. The pick order is passed as parameters. The idea here is, the arm reads the pick order of the samples, it then detects the samples through the camera connected. Based on the order, the arm picks each samples and places them in a predefined position for each colored sample. The pickorder is passed through the params file, which can be modified to any order of choice.
- ### Sequence followed:

  There are two nodes running: one for perception and one for pick and place. The perception node calls the */pc_filter/get_cluster_positions* service to get the color and position of each sample. After that, the samples positions are transformed into the correct reference frame, cleaned up, and published on the */detected_blocks* topic.

  The pick-and-place node subscribes to this topic so it can read the sample positions and colors. Using the given pick order, it then controls the arm to carry out the pick-and-place steps. The motion sequence is the same one used in pick_place moveIt's move_action client.

  Sequence(Brief):

  The robotic arm begins by moving close to the spot where it needs to pick up the sample, pausing just before reaching the *before_pick* in the code. It then glides carefully into the pick position. Once there, the gripper closes, indicating that the pick is complete.

  With the sample held, the arm lifts it smoothly to the *above_pick* position, making sure it's safely clear of any obstacles. It then moves to the *above_place* point, positioning itself above where the sample will be set down. From there, the arm lowers to the *place_point* and gently opens the gripper, releasing the sample into place. the *place_point* here is the *place_pos* and the z-axis of it is dynamically calculated based on the height of samples.
- ### Points taken into consideration:


  - lift_height - a constant value is given to lift the sample above the ground. This value is added to the z-value of a point.
  - place_pos - a constant value indicating the position where the samples need to be placed. It is currently pre-defined in the code and not provided through params.

## How to run(Make sure you are in folder Robotics_Projects):

Build and source

```bash
colcon build
source install/setup.bash
```

Command to run the application:

```bash
ros2 launch robot_launch control_only.launch.py robot_model:=rx200
```

The below command is used to change the order of picking the samples. This will help in avoiding to restart all the nodes and just triggers the perception node and publishes the new detected samples to pick.

Open a new terminal and just paste the below command, give the order of picking of samples as per your need and enter.

```bash
ros2 param set /pick_place_perception pick_order "[blue, yellow, red]"
```

## Folder Structure:

- labSample_sorting:
  - src/
    - ros2_perception/ros2_perception
      - Description: Code demonstrating the pick & place three colored samples - has two nodes, one for detecting the samples publishing their color&positions(perception Node) and other one where the movement of arm updates(client Node).
      - rx200_pick_place_perception.py(perception Node):
        - Description: This is a node that publishes the color and position of the samples to the Client Node
      - rx200_pick_place_client.py(client Node):
        - Description: Apart from the code done in the lab(initializing and defining constraints), this node subscribes to the perception node. A series of sequence as done in the pick_place has been used here to move the arms.
    - ros2_perception/config/rx200_pick_place.rviz
      - The custom rviz file which has all the nodes of perception and motion planning to visualize in rviz.
    - robot_launch/
      - Description: Launch file to run the code.
      - Purpose: Can be used to run along with any additional arguments passed through command line.
  - Video_Report:
    - Report.pdf: This is a report created by the team on the use-case of color-based laboratory sample sorting.
    - Video: A 2 mins video showcasing the use-case and small demo of the implementation.

