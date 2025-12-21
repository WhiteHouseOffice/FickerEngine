#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

// We still keep callbacks (for non-movement keys like toggle, etc.)
static void glfw_key_callback(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/) {
  if (action != GLFW_PRESS && action != GLFW_RELEASE) return;
  const bool down = (action == GLFW_PRESS);

  switch (key) {
    case GLFW_KEY_SPACE: Input::setKey(Input::KEY_SPACE, down); break;

    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL:
      Input::setKey(Input::KEY_CTRL, down);
      break;

    case GLFW_KEY_LEFT_SHIFT:
    case GLFW_KEY_RIGHT_SHIFT:
      Input::setKey(Input::KEY_SHIFT, down);
      break;

    case GLFW_KEY_F:
      Input::setKey(Input::KEY_F, down);
      break;

    default: break;
  }
}

static void glfw_cursor_pos_callback(GLFWwindow* /*window*/, double x, double y) {
  Input::onMouseMove(x, y);
}

static void glfw_focus_callback(GLFWwindow* /*window*/, int /*focused*/) {
  // Clear stuck keys/mouse when focus changes
  Input::resetAll();
}

static void PollMovementKeys(GLFWwindow* window) {
  // Polling prevents “stuck key” if a release event is missed
  Input::setKey(Input::KEY_W, glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
  Input::setKey(Input::KEY_A, glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
  Input::setKey(Input::KEY_S, glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
  Input::setKey(Input::KEY_D, glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
}

int main() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

  // Legacy/compat OpenGL so fixed-function debug rendering works
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);

  GLFWwindow* window = glfwCreateWindow(1280, 720, "FickerEngine", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // Capture mouse
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
  }

  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);

  glEnable(GL_DEPTH_TEST);

  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    // Keep movement keys in sync every frame (prevents stuck walking)
    PollMovementKeys(window);

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
