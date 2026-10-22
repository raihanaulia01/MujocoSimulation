#ifndef SIM_H
#define SIM_H

#include "control.hpp"
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

class Simulation {
public:
  mjModel *m = nullptr;
  mjData *d = nullptr;
  mjvScene scn;
  mjvCamera cam;
  mjvOption opt;
  mjrContext con;
  GLFWwindow *window = nullptr;

  Simulation();
  ~Simulation();

  bool initialize(const char *robotModelPath);
  void run(const RobotController &ctrl);
};

#endif