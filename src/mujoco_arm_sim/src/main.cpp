#include "ros/ros.h"
#include "callbacks.hpp"
#include "control.hpp"
#include "sim.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mujoco_arm_sim");
  ros::NodeHandle nh("~");

  std::string model_path;
  if (!nh.getParam("model_path", model_path)) {
    ROS_FATAL("Parameter model_path is missing.");
    return 1;
  }

  glfwSetErrorCallback(callbacks::glfw_error_callback);
  Simulation sim;

  if (!sim.initialize(model_path.c_str()))
    return 1;

  RobotController ctrl(sim.m);

  callbacks::initialize(&sim, &ctrl, sim.window);

  sim.run(ctrl);
  return 0;
}