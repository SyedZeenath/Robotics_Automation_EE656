## Project Details:

- Description: 


- Points taken into consideration:


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
        - Description: Assignment demonstrating a pick & place with perception - 
      - ros2_perception/params/rx200_params.yaml
        - The file to modify the positions and other parameters passed into the application.
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