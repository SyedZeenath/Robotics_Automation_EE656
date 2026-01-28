# Robotics_Automation_EE656
Embedded AI Team Project

## Folder structure (overview)

Project root
- Assignment_1_lab/
  - Description: Assignment workspace containing initial lab exercise.
  - Layout:
    - src/
      - rx200_moveit_control/
        - Description: MoveIt-based control demos for the RX200 manipulator.
        - Purpose: Move the arm from its base/home to a fixed target pose using MoveIt planners.
        - Typical contents:
          - ROS2 nodes (Python) for planning and execution
          - launch files (.py) to start MoveIt and robot controllers
      - robot_bringup/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.
  - Command used to run: 
  ```bash 
  ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual
  ```

- Assignment_1/
  - Description: Assignment workspace containing assignment on ROS2 Manipulation.
  - Layout:
    - src/
      - ros2_manipulation/ 
        - Description: Main assignment demonstrating a ROS2 manipulation (pick & place).
        - Objective:
          - Start from a known arm posture.
          - Move to a pick pose, grasp the object.
          - Move to a place pose, release the object.
          - Compute a trajectory composed of straight-line segments between waypoints derived from three reference points:
            - arm start pose
            - pick pose
            - place pose
        - Typical contents: (To Be Updated as and when required)
          - node(s) implementing trajectory generation (waypoints interpolation)
          - grasping/IK utilities and end-effector control
          - sample configuration files (YAML/JSON) with fixed poses used for testing
          - launch files and README describing how to run the demo
      - robot_launch/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.
    - scripts: helper scripts for build, test and demos
    - docs/ (optional): design notes, diagrams, experiment logs
   
- Assignment-2:
  - Description: Assignment 02 workspace containing ROS2 pick and stack cubes with perception logic
  - Layout:
    - src/
      - ros2_perception/ros2_perception
        - Description: Assignment demonstrating the pick & stack three colored blocks - has two nodes, one for detecting the blocks publishing their color&positions(perception Node) and other one where the movement of arm updates(client Node).
        - rx200_pick_place_perception.py(perception Node):
          - Description: This is a node that publishes the color and position of the blocks to the Client Node
        - rx200_pick_place_client.py(client Node):
           - Description: Apart from the code done in the lab(initializing and defining constraints), this node subscribes to the perception node. A series of sequence as done in the assignment_1 has been used here to move the arms.
      - ros2_perception/config/rx200_pick_place.rviz
        - The custom rviz file which has all the nodes of perception and motion planning to visualize in rviz.
      - robot_launch/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.


Notes
- Each folder should include a small README describing how to run its demos (dependencies, ros2 commands, launch usage).
- Keep pose/configuration values in separate files (YAML or JSON) where possible to make experiments reproducible.
- Use this top-level README to summarize repository purpose and provide quick start steps.
