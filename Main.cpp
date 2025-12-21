#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

static bool g_mouseCaptured = false;

// Movement key polling avoids stuck keys
static void PollMovementKeys(GLFWwindow* window) {
  Input::setKey(Input::KEY_W, glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
  Input::setKey(Input::KEY_A, glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
  Input::setKey(Input::KEY_S, glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
  Input::setKey(Input::KEY_D, glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
}

static void SetMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;

  if (captured) {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if (glfwRawMouseMotionSupported()) {
      glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    }
  } else {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
  }

  // Clear deltas to avoid a big jump on toggle
  Input::resetAll();

  // Recenter immediately so first frame delta is sane
  int w = 0, h = 0;
  glfwGetWindowSize(window, &w, &h);
  glfwSetCursorPos(window, w * 0.5, h * 0.5);
}

static void glfw_window_close_callback(GLFWwindow* window) {
  glfwSetWindowShouldClose(window, GLFW_TRUE);
}

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    if (!g_mouseCaptured) SetMouseCaptured(window, true);
  }
}

static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    if (g_mouseCaptured) {
      SetMouseCaptured(window, false);
    } else {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
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
  if (!focused && g_mouseCaptured) {
    // don’t get stuck captured when alt-tabbing
    SetMouseCaptured(window, false);
  }
  Input::resetAll();
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

  glEnable(GL_DEPTH_TEST);

  Engine::instance().init();

  // start released; click to capture
  SetMouseCaptured(window, false);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    PollMovementKeys(window);

    // ---- RELATIVE MOUSE BY RECENTERING ----
    if (g_mouseCaptured) {
      int w = 0, h = 0;
      glfwGetWindowSize(window, &w, &h);
      const double cx = w * 0.5;
      const double cy = h * 0.5;

      double mx = cx, my = cy;
      glfwGetCursorPos(window, &mx, &my);

      // dx/dy relative to center
      const float dx = static_cast<float>(mx - cx);
      const float dy = static_cast<float>(my - cy);

      // accumulate into Input
      Input::addMouseDelta(dx, dy);

      // warp back to center (prevents drift; gives stable deltas)
      glfwSetCursorPos(window, cx, cy);
    }

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
