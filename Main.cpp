#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

static bool g_captured = false;

static void PollMovementKeys(GLFWwindow* window) {
  Input::setKey(Input::KEY_W, glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
  Input::setKey(Input::KEY_A, glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
  Input::setKey(Input::KEY_S, glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
  Input::setKey(Input::KEY_D, glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
}

static void SetCaptured(GLFWwindow* window, bool captured) {
  g_captured = captured;

  if (captured) {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Optional: only works when cursor is disabled
    if (glfwRawMouseMotionSupported()) {
      glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    }

    // Avoid huge jump on first move after capture
    Input::resetMouse();
  } else {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
    Input::resetMouse();
  }
}

static void glfw_window_close_callback(GLFWwindow* window) {
  glfwSetWindowShouldClose(window, GLFW_TRUE);
}

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    if (!g_captured) SetCaptured(window, true);
  }
}

static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    if (g_captured) SetCaptured(window, false);
    else glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }

  if (action != GLFW_PRESS && action != GLFW_RELEASE) return;
  const bool down = (action == GLFW_PRESS);

  switch (key) {
    case GLFW_KEY_SPACE: Input::setKey(Input::KEY_SPACE, down); break;

    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL: Input::setKey(Input::KEY_CTRL, down); break;

    case GLFW_KEY_LEFT_SHIFT:
    case GLFW_KEY_RIGHT_SHIFT: Input::setKey(Input::KEY_SHIFT, down); break;

    case GLFW_KEY_F: Input::setKey(Input::KEY_F, down); break;

    default: break;
  }
}

static void glfw_focus_callback(GLFWwindow* window, int focused) {
  // If you alt-tab, release capture so input doesn't "stick/die"
  if (!focused && g_captured) SetCaptured(window, false);
  Input::resetAll();
}

static void glfw_cursor_pos_callback(GLFWwindow* /*window*/, double x, double y) {
  // ONLY feed deltas when captured
  if (!g_captured) return;
  Input::onMouseMove(x, y);
}

int main() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

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

  glfwSetWindowCloseCallback(window, glfw_window_close_callback);
  glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);

  glEnable(GL_DEPTH_TEST);

  Engine::instance().init();
  SetCaptured(window, false);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    PollMovementKeys(window);

    glClearColor(0.1f, 0.2f, 0.35f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Engine::instance().stepOnce();

    glfwSwapBuffers(window);
  }

  Engine::instance().shutdown();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
