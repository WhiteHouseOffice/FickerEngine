#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

// GLFW key callback -> engine Input
static void glfw_key_callback(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/) {
  const bool down = (action != GLFW_RELEASE);

  switch (key) {
    case GLFW_KEY_W: Input::setKey('W', down); break;
    case GLFW_KEY_A: Input::setKey('A', down); break;
    case GLFW_KEY_S: Input::setKey('S', down); break;
    case GLFW_KEY_D: Input::setKey('D', down); break;
    case GLFW_KEY_SPACE: Input::setKey(' ', down); break;

    // Map Ctrl to a key code you can query if you want (choose one; 0x11 is common)
    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL:
      Input::setKey(0x11, down);
      break;

    default: break;
  }
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
