# Robotics_Projects

The projects have been done as part of the course assignments during MSc Robotics and Embedded AI.

## Folder structure (overview)

Project root

- pick_place/

  - Description: Workspace containing pick and place on ROS2 Manipulation.
  - Layout:
    - src/
      - ros2_manipulation/
        - Description: Code demonstrating a ROS2 manipulation (pick & place).
        - Objective:
          - Start from a known arm posture.
          - Move to a pick pose, grasp the object.
          - Move to a place pose, release the object.
          - Compute a trajectory composed of straight-line segments between waypoints derived from three reference points:
            - arm start pose
            - pick pose
            - place pose
        - Typical contents:
          - node(s) implementing trajectory generation (waypoints interpolation)
          - grasping/IK utilities and end-effector control
          - sample configuration files (YAML/JSON) with fixed poses used for testing
          - launch files and README describing how to run the demo
      - robot_launch/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.
    - scripts: helper scripts for build, test and demos
    - docs/ (optional): design notes, diagrams, experiment logs

- pick_stack:

  - Description: Workspace containing ROS2 pick and stack cubes with perception logic
  - Layout:
    - src/
      - ros2_perception/ros2_perception
        - Description: Code demonstrating the pick & stack of three colored blocks - has two nodes, one for detecting the blocks publishing their color & positions(perception Node) and other one where the movement of arm updates(client Node).
        - rx200_pick_place_perception.py(perception Node):
          - Description: This is a node that publishes the color and position of the blocks to the Client Node
        - rx200_pick_place_client.py(client Node):
          - Description: This node subscribes to the perception node. A series of sequence as done in the 'pick_place' has been used here to move the arms.
      - ros2_perception/config/rx200_pick_place.rviz
        - The custom rviz file which has all the nodes of perception and motion planning to visualize in rviz.
      - robot_launch/
        - Description: Launch file to run the code.
        - Purpose: Can be used to run along with any additional arguments passed through command line.

- labSample_sorting:
  - Description: Similar code as above, just changed few final positions, this project basically indicates a real-world use case in sorting blood samples in a laboratory, titled: A Modular Robotic System for Colour-Based Laboratory Sample Sorting
  - Layout:
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


Notes

- Each folder should include a small README describing how to run its demos (dependencies, ros2 commands, launch usage).
- Keep pose/configuration values in separate files (YAML or JSON) where possible to make experiments reproducible.
- Use this top-level README to summarize repository purpose and provide quick start steps.
