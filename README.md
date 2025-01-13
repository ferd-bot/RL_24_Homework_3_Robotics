# **RL24_HW_3**
**Authors**: Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## Overview

This is **Homework 3**, where the objective is to implement a vision-based controller for a 7-degrees-of-freedom robotic manipulator arm within the Gazebo simulation environment.

---

## **Instructions**

1. Clone the repository from GitHub:  
   ```bash
   cd src
   git clone -b REV_3 https://github.com/ferd-bot/RL_24_Homework_3_Robotics.git .
   ```
   **Important**:  
   The `git clone` command with the dot (`.`) works only if the target directory is empty.  
   - If not, you can remove extra files using:
     ```bash
     rm -rf *
     ```
   - Alternatively, clone the repository normally (without the dot) and manually move the files from the `RL_24_Homework_2_Robotics` folder to the `src` directory.

2. Configure and build all packages in the ROS2 workspace:  
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

---

## Launching the Simulation in Gazebo

### Start the IIWA Robot with Velocity Controller and Camera Enabled
To launch the Gazebo simulation with the effort controller and enable the camera:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true" use_vision:="true"
```

- **Note**: If the camera is not required, set `use_vision:=false`:
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true" use_vision:="false"
  ```

---

### Start the IIWA Robot with Velocity Controller, Camera, and Spherical Object Enabled
To launch the Gazebo simulation with the effort controller, camera, and spherical object:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" use_vision:="true" spherical_object:="true"
```

To move the manipulator toward the sphere (position control), open a new terminal and run:
```bash
ros2 topic pub /iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "data: [1.7, 2.0, 0.0, 2.0, 0.0, -1.5, 0.0]"
```

---

## OpenCV Integration

### Launch the OpenCV Node
To use OpenCV functionalities, run the following command in a new terminal:
```bash
ros2 run ros2_opencv ros2_opencv_node
```

---

To visualize the image processed by OpenCV, open `rqt` in another terminal and subscribe to the topic `/processed_image`:
```bash
rqt
```

---

## ArUco Marker Integration

To launch Gazebo in velocity mode with ArUco tags enabled:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true" use_vision:="true" aruco_tag:="true"
```

This command also opens RViz with a pre-configured setup for the `aruco_single` node to detect the marker.

- To perform the **Positioning Task** in velocity mode, open another terminal and run:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p task:=positioning
  ```

- To perform the **Look-at-Point Task** in velocity mode, open another terminal and run:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p task:=look-at-point
  ```

---

## Executing Trajectories in Effort Mode

You can execute the four trajectories defined in the previous homework using effort mode.  
To start Gazebo in effort mode:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true" use_vision:="true" aruco_tag:="true"
```

To execute the trajectories, open a new terminal and run one of the following commands depending on the desired control mode:

- **Effort Mode in Joint Space**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort -p control_space:=joint_space
  ```

- **Effort Mode in Operational Space**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort -p control_space:=operational_space
  ```

---

## **Available Trajectories**

The following trajectories are supported, all controlled in **Effort Mode** with Joint Space as the default.  
Trajectories are numbered from 0 to 3:

1. **Linear with Trapezoidal Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort -p control_space:=joint_space
   ```

2. **Linear with Cubic Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 1 --ros-args -p cmd_interface:=effort -p control_space:=joint_space
   ```

3. **Circular with Trapezoidal Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 2 --ros-args -p cmd_interface:=effort -p control_space:=joint_space
   ```

4. **Circular with Cubic Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 3 --ros-args -p cmd_interface:=effort -p control_space:=joint_space
   ```

To execute trajectories in **Operational Space**, simply set `control_space:=operational_space`.

---

## **Additional Notes**

A new topic `/torque_plot` has been added.  
This topic allows you to visualize the torques applied to the manipulator's joints.

To monitor the topic in the terminal:
```bash
ros2 topic echo /torque_plot
```

Alternatively, use `rqt_plot` for graphical visualization:
```bash
rqt
```

In `rqt`, set up the plotting plugin and add the following data points for each joint:
```
/torque_plot/data[0]
/torque_plot/data[1]
/torque_plot/data[2]
/torque_plot/data[3]
/torque_plot/data[4]
/torque_plot/data[5]
/torque_plot/data[6]
```

**Additional Topics**:
- Visualize the detected ArUco pose:
  ```bash
  ros2 topic echo /aruco_detect/pose
  ```

---

## **Videos**
For simplicity, only videos demonstrating **Effort-Based Control in Operational Space with Gazebo** are included:
- [Video URL Placeholder]

