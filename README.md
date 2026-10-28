# Mujoco Simulation
A Ros-based Mujoco Simulation project controlling a primitive unrealistic robotic arm. This Project was made to fullfill an assignment from the Abinara-1 Research Team.

## Simulation Preview
<img width="2560" height="1600" alt="Simulation Preview" src="https://github.com/user-attachments/assets/eab42473-9dfa-414e-8b7a-d62c3294f07f" />

## Overview
This project consists of:
- A mujoco robotic arm model
- A GLFW window for visualization and keyboard teleop
- ROS Communication for receiving and publishing joint states
- A position-only kinematics connected through ROS

## Requirements
- Ubuntu with ROS Noetic
- C++17
- CMake
- GLFW
- [MuJoCo](https://mujoco.org/)
- `roscpp` and `sensor_msgs` (both are ROS packages)

The configured `CMakeLists.txt` file assumes that you have Mujoco installed in the following paths:
```text
/usr/local/include
/usr/local/lib/libmujoco.so.3.2.7
```
Update these paths if you installed Mujoco somewhere else. 

## Running the program
### 1. Start ROS
```bash
roscore
```
Keep this terminal running.

### 2. Build and prepare the workspace
Make sure you are in the project's root directory (e.g. `~/Projects/MujocoSimulation`), then run:
```bash
catkin_make
```
And this:
```bash
source devel/setup.bash
```
Or if you use zsh:
```zsh
source devel/setup.zsh
```

### 3. Start the simulation
In a new terminal:
```bash
roslaunch mujoco_arm_sim arm_sim.launch
```

### 4. Optional: Running the triangle trajectory
Run this in a new terminal:
```bash
rosrun mujoco_arm_sim triangle_trajectory
```
This node publishes hardcoded joint positions to `/joint_command` to create a triangular-like shape with the end effector.

### 5. Optional: Running the kinematics node
Run this in a new terminal:
```bash
rosrun mujoco_arm_sim kinematics_node
```

## Controls

### Keyboard Controls
| Joint | Increment angle | Decrement angle | Limit (rad) |
| ----- | ----- | ----- | ----- |
| Joint 1   | `A`   | `Z`   | -3.14 - 3.14 |
| Joint 2   | `S`   | `X`   | -1.57 - 1.57 |
| Joint 3   | `D`   | `C`   | -2.35 - 2.35 |
| Joint 4   | `F`   | `V`   | -1.57 - 1.57 |
| Joint 5   | `G`   | `B`   | -3.14 - 3.14 |

### Camera Navigation
- **Rotate** : Left click + drag
- **Pan** : Right click + drag
- **Zoom** : Middle click + drag or scroll wheel


## Kinematics Node
`kinematics_node` provides a real-time Forward Kinematics (FK) and Inverse Kinematics (IK) for the robot arm. \
This uses DH parameters to calculate the end-effector's position and implements a Damped Least Squares (DLS) method to compute the IK.

### DH Parameters
|  | a | alpha | d | theta |
| ---- | - | ----- | - | ----- |
| 1 | 0 | -90° | 0.15 | q1 |
| 2 | 0.45 | 0 | -0.05 | q2 - 90° |
| 3 | 0.45 | 0 | 0 | q3 |
| 4 | 0 | 90° | -0.1 | q4 + 90° |
| 5 | 0 | 0 | 0.05 | q5 |

> Note:
> This kinematics node currently only computes the positional values, no rotational input is currently supported.

## ROS Topics

| Topic | Direction | Message type | Description |
| ----- | --------- | ------------ | ----------- |
| `/joint_command` | Published by controller nodes | `sensor_msgs/JointState` | Target joint positions for the simulation |
| `/joint_states` | Published by the MuJoCo simulation | `sensor_msgs/JointState` | Current simulated joint positions |
| `/end_effector` | Published by kinematics | `geometry_msgs/PointStamped.h` | Current end-effector position in world space |
| `/target_point` | Subscribed by kinematics | `geometry_msgs/PointStamped.h` | Used to set target position of the end-effector |

### Inspecting Topics
List active topics:
```bash
rostopic list
```

Inspect a topic:
```bash
rostopic echo <topic>
```
> Replace `<topic>`, including the angle brackets, with the topic name (`/joint_command` or `/joint_states`)

## Message Types
`/joint_command` and `/joint_states` use `sensor_msgs/JointState`. \
The fields are:
```text
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```
This project only uses the `name` and `position` fields.

`end_effector` and `/target_point` use `geometry_msgs/PointStamped.h` \
The fields are:
```text
std_msgs/Header header
geometry_msgs/Point point:
  x,
  y,
  z
```

### Joint Names
The joint names as defined in the model are:
```text
joint_1_hip
joint_2_shoulder
joint_3_elbow
joint_4_wrist
joint_5_roll
```

## Example Commands
Run these commands in a separate terminal. 

A joint command must contain exactly five joint positions, e.g.:
```bash
rostopic pub /joint_command sensor_msgs/JointState "{name: ['joint_1_hip', 'joint_2_shoulder', 'joint_3_elbow', 'joint_4_wrist', 'joint_5_roll'], position: [0.0, -1.40, 0.70, 0.0, 0.0]}"
```

Publish a specific joint position to the kinematics node:
```bash
rostopic pub -1 /target_point geometry_msgs/PointStamped "
point:
  x: 0.3
  y: 0.2
  z: 0.4"
```