# NewRelease_Homework_3: Enhanced Codebase

## Overview

The **NewRelease_Homework_3** branch introduces an improved version of the code, developed after the homework deadline. A dynamic vision-based controller has been implemented, allowing the manipulator to operate in **effort mode**, align itself to a reference tag, and execute two types of trajectories while tracking the tag:

1. **Linear Trajectory with Trapezoidal Velocity Profile**
2. **Circular Trajectory with Trapezoidal Velocity Profile**

---

## Setup Instructions

### 1. Clone the Repository
Download the repository from GitHub:
```bash
git clone -b REV_2 https://github.com/ferd-bot/RL_24_Homewrok_3_Robotics.git
```

### 2. Configure and Build the Workspace
To configure and build the workspace:
```bash
colcon build
source install/setup.bash
```

**Note**: The repository download includes extra files. Manually remove unnecessary files and move the required ones into the `src` folder.
Additionally, ensure that the environment variable GZ_SIM_RESOURCE_PATH includes the path to the Gazebo models required by the simulation. If this is not configured, you may encounter errors like Unable to find uri[model://...]. To resolve this, execute the following commands:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/models
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

---

## Launching the Simulation in Gazebo

### Launch the IIWA Robot with Velocity Controller and Camera Enabled
To start the Gazebo simulation with the effort controller and enable the camera:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true" use_vision:="true"
```

- **Note**: If the camera is not needed, set `use_vision:=false`:
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true" use_vision:="false"
  ```

---

## OpenCV Integration

### Launch the OpenCV Node
To use OpenCV functionalities, run the following command:
```bash
ros2 run ros2_opencv ros2_opencv_node
```

---

## ArUco Marker Integration

### Establish Marker Tracking
To integrate with ArUco markers, run the following command:
```bash
ros2 run aruco_ros single --ros-args   -r /image:=/videocamera   -r /camera_info:=/videocamera_info   -p marker_id:=201   -p marker_size:=0.1   -p reference_frame:=camera_link   -p marker_frame:=aruco_marker_frame   -p camera_frame:=camera_link
```

---

## Executing Trajectories

- **Linear Trajectory**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p trajectory:=linear
  ```

- **Circular Trajectory**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p trajectory:=circular
  ```

---

## Notes

- Videos demonstrating the manipulator's functionality will be attached for reference.
- https://youtu.be/JdOeHxABXvs - Trajectory Linear Trapezoidal
- https://youtu.be/ppoZ1WKf1oo - Trajectory Circular Trapezoidal
