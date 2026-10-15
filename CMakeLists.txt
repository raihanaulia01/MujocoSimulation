cmake_minimum_required(VERSION 3.16)
project(mujoco-arm-sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
# pkg_search_module(MUJOCO mujoco)

set(MUJOCO_INCLUDE_DIRS /usr/local/include/mujoco/)
set(MUJOCO_LIBRARIES /usr/local/lib/libmujoco.so.3.2.7)

include_directories(include)
add_library(simlibs src/callbacks/callbacks.cpp src/control/control.cpp src/sim/sim.cpp)
target_include_directories(simlibs PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/callbacks
  ${CMAKE_CURRENT_SOURCE_DIR}/src/control
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sim
  ${GLFW_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS}
)

add_executable(sim src/main.cpp)
target_link_libraries(sim PRIVATE simlibs ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES})