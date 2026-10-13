#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

mjModel *m = nullptr;
mjData *d = nullptr;
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {}

void mouse_move_callback(GLFWwindow *window, double xpos, double ypos) {}

void mouse_scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
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

  while (!glfwWindowShouldClose(window)) {
    mj_step(m, d);

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