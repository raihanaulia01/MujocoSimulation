#include "callbacks.hpp"
#include <cstdio>

namespace callbacks {

static Simulation *sim = nullptr;
static RobotController *ctrl = nullptr;

static bool buttonLeft = false, buttonMiddle = false, buttonRight = false;
static double lastx = 0, lasty = 0;

void initialize(Simulation *simulation, RobotController *controller,
                GLFWwindow *window) {
  sim = simulation;
  ctrl = controller;

  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, mouse_move_callback);
  glfwSetScrollCallback(window, mouse_scroll_callback);
  glfwSetKeyCallback(window, key_callback);
}

void glfw_error_callback(int error_code, const char *desc) {
  fprintf(stderr, "GLFW Error Code %d:\n%s\n", error_code, desc);
}

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    buttonLeft = (action == GLFW_PRESS)
                     ? true
                     : (action == GLFW_RELEASE ? false : buttonLeft);
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    buttonMiddle = (action == GLFW_PRESS)
                       ? true
                       : (action == GLFW_RELEASE ? false : buttonMiddle);
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
    buttonRight = (action == GLFW_PRESS)
                      ? true
                      : (action == GLFW_RELEASE ? false : buttonRight);

  glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move_callback(GLFWwindow *window, double xpos, double ypos) {
  if (!buttonLeft && !buttonRight && !buttonMiddle) {
    lastx = xpos;
    lasty = ypos;
    return;
  }

  int fbw = 0, fbh = 1;
  glfwGetFramebufferSize(window, &fbw, &fbh);
  if (fbh <= 0)
    fbh = 1;

  double dx = (xpos - lastx) / (double)fbh;
  double dy = (ypos - lasty) / (double)fbh;
  lastx = xpos;
  lasty = ypos;

  if (buttonLeft) {
    mjv_moveCamera(sim->m, mjMOUSE_ROTATE_H, dx, dy, &sim->scn, &sim->cam);
  } else if (buttonRight) {
    mjv_moveCamera(sim->m, mjMOUSE_MOVE_H, dx, dy, &sim->scn, &sim->cam);
  } else if (buttonMiddle) {
    mjv_moveCamera(sim->m, mjMOUSE_ZOOM, dx, dy, &sim->scn, &sim->cam);
  }
}

void mouse_scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  mjv_moveCamera(sim->m, mjMOUSE_ZOOM, 0, 0.025 * yoffset, &sim->scn,
                 &sim->cam);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mods) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT)
    return;

  double step = 0.05;
  switch (key) {
  case GLFW_KEY_A:
    ctrl->jointPos[0] += step;
    break;
  case GLFW_KEY_Z:
    ctrl->jointPos[0] -= step;
    break;
  case GLFW_KEY_S:
    ctrl->jointPos[1] += step;
    break;
  case GLFW_KEY_X:
    ctrl->jointPos[1] -= step;
    break;
  case GLFW_KEY_D:
    ctrl->jointPos[2] += step;
    break;
  case GLFW_KEY_C:
    ctrl->jointPos[2] -= step;
    break;
  case GLFW_KEY_F:
    ctrl->jointPos[3] += step;
    break;
  case GLFW_KEY_V:
    ctrl->jointPos[3] -= step;
    break;
  }

  ctrl->checkJointLimits();
  ctrl->printJointPos();
}

} // namespace callbacks