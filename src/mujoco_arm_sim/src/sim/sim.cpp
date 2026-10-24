#include "sim.hpp"

#include <cstdio>

Simulation::Simulation() {
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
}

Simulation::~Simulation() {
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();
}

bool Simulation::initialize(const char *robotModelPath) {
  char error[1000] = "Couldn't load model";
  m = mj_loadXML(robotModelPath, nullptr, error, sizeof(error));
  if (!m) {
    fprintf(stderr, "%s\n", error);
    return false;
  }
  d = mj_makeData(m);

  if (!glfwInit()) {
    fprintf(stderr, "glfwInit() failed\n");
    return false;
  }

  window = glfwCreateWindow(1600, 900, "Arm sim", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "glfwCreateWindow() failed\n");
    return false;
  }
  glfwMakeContextCurrent(window);

  mjv_makeScene(m, &scn, 6700);
  mjr_makeContext(m, &con, mjFONTSCALE_100);
  return true;
}

void Simulation::run(const RobotController &ctrl, RosInterface &rosInterface) {
  while (!glfwWindowShouldClose(window) && ros::ok()) {
    ros::spinOnce();

    ctrl.applyJointPos(d);
    mj_step(m, d);

    rosInterface.publishJointPos(d);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}