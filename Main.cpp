#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

static bool g_mouseCaptured = false;

static void SetMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;

  if (captured) {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Raw mouse is nice when it works; safe to enable only if supported.
    if (glfwRawMouseMotionSupported()) {
      glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    }

    // Reset so the first delta after capture isn't huge / one-sided
    Input::resetAll();
  } else {
    // Release cursor back to OS
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // Disable raw mouse motion if it was on
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);

    Input::resetAll();
  }
}

// X button always closes
static void glfw_window_close_callback(GLFWwindow* window) {
  glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// Click inside window = capture mouse
static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    if (!g_mouseCaptured) {
      SetMouseCaptured(window, true);
    }
  }
}

// ESC = release mouse (and also allow quitting if you want)
static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    if (g_mouseCaptured) {
      SetMouseCaptured(window, false);
    } else {
      // If already released, ESC closes (optional, but convenient)
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    return;
  }

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
  // Only process deltas when captured (prevents weird drift when free)
  if (!g_mouseCaptured) return;
  Input::onMouseMove(x, y);
}

static void glfw_focus_callback(GLFWwindow* window, int focused) {
  // If we lose focus, release capture so you don't get stuck.
  if (!focused && g_mouseCaptured) {
    SetMouseCaptured(window, false);
  }
  // Clear states on focus change to avoid stuck keys
  Input::resetAll();
}

static void PollMovementKeys(GLFWwindow* window) {
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

  // Legacy/compat OpenGL for fixed-function debug drawing
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
  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);

  glEnable(GL_DEPTH_TEST);

  Engine::instance().init();

  // Start NOT captured: click inside to capture
  SetMouseCaptured(window, false);

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
