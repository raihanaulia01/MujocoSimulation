cmake_minimum_required(VERSION 3.0.2)
project(mujoco_arm_sim)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
)

set(MUJOCO_INCLUDE_DIRS /usr/local/include/)
set(MUJOCO_LIBRARIES /usr/local/lib/libmujoco.so.3.2.7)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mujoco_arm_node
  src/main.cpp
  src/callbacks/callbacks.cpp
  src/control/control.cpp
  src/sim/sim.cpp
  src/ros_interface/ros_interface.cpp
)

target_link_libraries(mujoco_arm_node ${catkin_LIBRARIES} ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES})
add_dependencies(mujoco_arm_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(triangle_trajectory src/triangle_trajectory/triangle_trajectory.cpp)
target_link_libraries(triangle_trajectory ${catkin_LIBRARIES})
add_dependencies(triangle_trajectory ${catkin_EXPORTED_TARGETS})

add_executable(kinematics_node src/kinematics/kinematics_node.cpp)
target_link_libraries(kinematics_node ${catkin_LIBRARIES})
add_dependencies(kinematics_node ${catkin_EXPORTED_TARGETS})