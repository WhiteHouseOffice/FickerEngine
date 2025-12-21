#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

static void glfw_key_callback(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/) {
  const bool down = (action != GLFW_RELEASE);

  switch (key) {
    case GLFW_KEY_W: Input::setKey(Input::KEY_W, down); break;
    case GLFW_KEY_A: Input::setKey(Input::KEY_A, down); break;
    case GLFW_KEY_S: Input::setKey(Input::KEY_S, down); break;
    case GLFW_KEY_D: Input::setKey(Input::KEY_D, down); break;

    case GLFW_KEY_SPACE: Input::setKey(Input::KEY_SPACE, down); break;

    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL:
      Input::setKey(Input::KEY_CTRL, down);
      break;

    case GLFW_KEY_LEFT_SHIFT:
    case GLFW_KEY_RIGHT_SHIFT:
      Input::setKey(Input::KEY_SHIFT, down);
      break;

    default: break;
  }
}

static void glfw_cursor_pos_callback(GLFWwindow* /*window*/, double x, double y) {
  Input::onMouseMove(x, y);
}

int main() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

  GLFWwindow* window =
      glfwCreateWindow(1280, 720, "FickerEngine", nullptr, nullptr);

  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glEnable(GL_DEPTH_TEST);

  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);

  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    glClearColor(0.1f, 0.2f, 0.35f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Engine::instance().stepOnce();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  Engine::instance().shutdown();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
