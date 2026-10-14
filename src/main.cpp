#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <iostream>

mjModel *m = nullptr;
mjData *d = nullptr;
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;

bool buttonLeft = false, buttonMiddle = false, buttonRight = false;
double lastx = 0, lasty = 0;

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    buttonLeft = (action == GLFW_PRESS) ? true : (action == GLFW_RELEASE ? false : buttonLeft);
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    buttonMiddle = (action == GLFW_PRESS) ? true : (action == GLFW_RELEASE ? false : buttonMiddle);
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
    buttonRight = (action == GLFW_PRESS) ? true : (action == GLFW_RELEASE ? false : buttonRight);

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
  if (fbh <= 0) fbh = 1;

  double dx = (xpos - lastx) / (double)fbh;
  double dy = (ypos - lasty) / (double)fbh;
  lastx = xpos;
  lasty = ypos;

  if (buttonLeft) {
    mjv_moveCamera(m, mjMOUSE_ROTATE_H, dx, dy, &scn, &cam);
  } else if (buttonRight) {
    mjv_moveCamera(m, mjMOUSE_MOVE_H, dx, dy, &scn, &cam);
  } else if (buttonMiddle) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, dx, dy, &scn, &cam);
  }
}

void mouse_scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.025 * yoffset, &scn, &cam);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mods) {
  if (action != GLFW_PRESS)
    return;
}

void glfw_error_callback(int error_code, const char *desc) {
  fprintf(stderr, "GLFW Error Code %d:\n%s\n", error_code, desc);
}

int main() {
  char error[1000] = "Couldn't load model";
  m = mj_loadXML("../models/arm.xml", nullptr, error, sizeof(error));

  if (!m) {
    fprintf(stderr, "%s\n", error);
    return 1;
  }

  d = mj_makeData(m);
  glfwSetErrorCallback(glfw_error_callback);

  if (!glfwInit()) {
    fprintf(stderr, "glfwInit() failed\n");
    return 1;
  }

  GLFWwindow *window = glfwCreateWindow(1600, 900, "Arm sim", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "glfwCreateWindow() failed\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, mouse_move_callback);
  glfwSetScrollCallback(window, mouse_scroll_callback);
  glfwSetKeyCallback(window, key_callback);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scn, 6700);
  mjr_makeContext(m, &con, mjFONTSCALE_100);

  // TODO add lastjointpos so it only updates when joint pos changes?
  while (!glfwWindowShouldClose(window)) {
    mj_step(m, d);
    for (int i = 0; i < 4; i++) {
      std::cout << "Joint " << i+1 << ": " << d->qpos[i] << std::endl;
    }
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();
}