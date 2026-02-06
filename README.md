# Robotics_Projects

The projects have been done as part of the course assignments during MSc Robotics and Embedded AI.

## Projects Overview

#### pick_place:

Workspace that includes implementation of pick and place of an object using just moveit and fixed points for the robotic arm to pick and place the object.

#### pick_stack:

Workspace that includes implementation of picking and stacking of 3 cubes. The cubes are detected through the camera that is attached to the wrist of the robotic arm. The detection done here is through pointclouds using interbotix libraries.

#### labSample_sorting:

This project basically indicates a real-world use case in sorting blood samples in a laboratory, titled: A Modular Robotic System for Colour-Based Laboratory Sample Sorting. It uses similar code as above, just changed the final place positions and few constraints.

*Check 'Video_Report' folder to get an idea on the implementation*

## Hardware and Software Stack

#### Hardware:

- A 6-DOF robotic manipulator
- Overhead RGB camera
- Standard colour-coded laboratory test containers

#### Software Stack:

- ROS2 Humble middleware
- Python 3.10
- Interbotix libraries for image processing
- MoveIt2 for motion planning
