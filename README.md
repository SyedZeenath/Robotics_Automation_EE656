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

Notes
- Each folder should include a small README describing how to run its demos (dependencies, ros2 commands, launch usage).
- Keep pose/configuration values in separate files (YAML or JSON) where possible to make experiments reproducible.
- Use this top-level README to summarize repository purpose and provide quick start steps.
