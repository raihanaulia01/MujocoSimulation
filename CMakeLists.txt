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
# add_library(simlibs)
add_executable(sim src/main.cpp)
target_include_directories(sim PRIVATE $(MUJOCO_INCLUDE_DIRS) $(GLFW_INCLUDE_DIRS))
target_link_libraries(sim PRIVATE ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES})
# target_link_libraries(sim simlibs)