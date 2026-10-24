# Mujoco Simulation
A Ros-based Mujoco Simulation project controlling a primitive unrealistic robotic arm. This Project was made to fullfill an assignment from the Abinara-1 Research Team.

## Overview
This project consists of:
- A mujoco robotic arm model
- A GLFW window for visualization and keyboard teleop
- ROS Communication for receiving and publishing joint states

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


## ROS Topics

| Topic | Direction | Message type | Description |
| ----- | --------- | ------------ | ----------- |
| `/joint_command` | Published by controller nodes | `sensor_msgs/JointState` | Target joint positions for the simulation |
| `/joint_states` | Published by the MuJoCo simulation | `sensor_msgs/JointState` | Current simulated joint positions |

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
Both ROS Topics use `sensor_msgs/JointState`. \
The fields are:
```text
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```
This project only uses the `name` and `position` fields.

### Joint Names
The joint names as defined in the model are:
```text
joint_1_hip
joint_2_shoulder
joint_3_elbow
joint_4_wrist
joint_5_roll
```

A joint command must contain exactly five joint positions, e.g.:
```bash
rostopic pub /joint_command sensor_msgs/JointState "{name: ['joint_1_hip', 'joint_2_shoulder', 'joint_3_elbow', 'joint_4_wrist', 'joint_5_roll'], position: [0.0, -1.40, 0.70, 0.0, 0.0]}"
```

