#include "callbacks.hpp"
#include "control.hpp"
#include "sim.hpp"

int main() {
  glfwSetErrorCallback(callbacks::glfw_error_callback);
  Simulation sim;

  if (!sim.initialize("src/mujoco_arm_sim/models/arm.xml"))
    return 1;

  RobotController ctrl(sim.m);

  callbacks::initialize(&sim, &ctrl, sim.window);

  sim.run(ctrl);
  return 0;
}