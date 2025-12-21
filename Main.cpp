#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

// --- Callbacks ----------------------------------------------------

static void glfw_window_close_callback(GLFWwindow* window) {
  // Make the X button ALWAYS close the loop.
  glfwSetWindowShouldClose(window, GLFW_TRUE);
}

static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  // ESC always exits (backup in case X is flaky)
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }

  // Only care about press/release
  if (action != GLFW_PRESS && action != GLFW_RELEASE) return;
  const bool down = (action == GLFW_PRESS);

  // Non-movement keys can be handled here
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

static void glfw_focus_callback(GLFWwindow* /*window*/, int focused) {
  // Reset on focus loss/gain so keys don't stick and mouse delta doesn't explode.
  // BUT only do it when focus CHANGES.
  (void)focused;
  Input::resetAll();
}

static void glfw_cursor_enter_callback(GLFWwindow* /*window*/, int entered) {
  // When cursor re-enters, reset mouse accumulator so first delta isn't huge.
  if (entered) Input::resetAll();
}

static void PollMovementKeys(GLFWwindow* window) {
  // Polling avoids "stuck key" if a release event is missed.
  Input::setKey(Input::KEY_W, glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
  Input::setKey(Input::KEY_A, glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
  Input::setKey(Input::KEY_S, glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
  Input::setKey(Input::KEY_D, glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
}

// --- Main ---------------------------------------------------------

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

  // Hard-wire close behavior
  glfwSetWindowCloseCallback(window, glfw_window_close_callback);

  // Input callbacks
  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);
  glfwSetCursorEnterCallback(window, glfw_cursor_enter_callback);

  // Capture mouse (required for FPS-style looking)
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // Raw mouse is nice, but can be flaky in some setups; only enable if supported.
  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
  }

  // IMPORTANT: Reset after capture so the first mouse delta isn't insane
  Input::resetAll();

  glEnable(GL_DEPTH_TEST);

  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    // MUST pump events every frame for mouse + X button to work
    glfwPollEvents();

    // Keep movement keys sane every frame
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
