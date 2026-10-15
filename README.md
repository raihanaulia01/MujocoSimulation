# Mujoco Simulation
A Mujoco Simulation project made with GLFW, C++, and CMAKE, controlling a primitive unrealistic robotic arm. This Project was made to fullfill an assignment from the Abinara-1 Research Team.

## Requirements
- C++17
- GLFW
- [MuJoCo](https://mujoco.org/)
- CMake

## Running the program
```bash
mkdir build
cd build
cmake ..
make
./sim
```

## Controls

### Keyboard Controls
| Joint | Increment angle | Decrement angle | Limit (rad) |
| ----- | ----- | ----- | ----- |
| Joint 1   | `A`   | `Z`   | -3.14 - 3.14 |
| Joint 2   | `S`   | `X`   | -1.57 - 1.57 |
| Joint 3   | `D`   | `C`   | -2.35 - 2.35 |
| Joint 4   | `F`   | `V`   | -1.57 - 1.57 |

### Camera Navigation
- **Rotate** : Left click + drag
- **Pan** : Right click + drag
- **Zoom** : Middle click + drag or scroll wheel