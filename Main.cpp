#include <cstdio>
#include <cmath>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include "core/Engine.h"
#include "core/Input.h"

static bool g_mouseCaptured = false;
static int  g_winW = 960;
static int  g_winH = 540;

static bool g_ignoreNextWarpEvent = false;

static void glfw_window_size_callback(GLFWwindow* /*window*/, int w, int h) {
  g_winW = (w > 1) ? w : 1;
  g_winH = (h > 1) ? h : 1;
}

static void setMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;
  Input::resetMouse();

  // ✅ WSLg-safe: do NOT use GLFW_CURSOR_DISABLED (can bias deltas)
  // Use NORMAL (or HIDDEN) + manual warp.
  glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_HIDDEN : GLFW_CURSOR_NORMAL);

  // Seed cursor to center on capture
  if (captured) {
    const double cx = g_winW * 0.5;
    const double cy = g_winH * 0.5;
    g_ignoreNextWarpEvent = true;
    glfwSetCursorPos(window, cx, cy);
  }
}

static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  const bool down = (action != GLFW_RELEASE);

  if (key == GLFW_KEY_ESCAPE && down) {
    setMouseCaptured(window, false);
    return;
  }

  switch (key) {
    case GLFW_KEY_W: Input::setKey(Input::KEY_W, down); break;
    case GLFW_KEY_A: Input::setKey(Input::KEY_A, down); break;
    case GLFW_KEY_S: Input::setKey(Input::KEY_S, down); break;
    case GLFW_KEY_D: Input::setKey(Input::KEY_D, down); break;
    case GLFW_KEY_SPACE: Input::setKey(Input::KEY_SPACE, down); break;

    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL: Input::setKey(Input::KEY_CTRL, down); break;

    case GLFW_KEY_LEFT_SHIFT:
    case GLFW_KEY_RIGHT_SHIFT: Input::setKey(Input::KEY_SHIFT, down); break;

    case GLFW_KEY_F: Input::setKey(Input::KEY_F, down); break;

    default: break;
  }
}

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    setMouseCaptured(window, true);
  }
}

static void glfw_focus_callback(GLFWwindow* window, int focused) {
  if (!focused) {
    setMouseCaptured(window, false);
    Input::resetAll();
  } else {
    Input::resetMouse();
    g_ignoreNextWarpEvent = false;
  }
}

int main() {
  if (!glfwInit()) {
    std::printf("Failed to initialize GLFW\n");
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  GLFWwindow* window = glfwCreateWindow(960, 540, "FickerEngine", nullptr, nullptr);
  if (!window) {
    std::printf("Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);
  glfwSetWindowSizeCallback(window, glfw_window_size_callback);

  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // ✅ Manual mouse delta capture (WSLg-safe)
    if (g_mouseCaptured) {
      double x = 0.0, y = 0.0;
      glfwGetCursorPos(window, &x, &y);

      const double cx = g_winW * 0.5;
      const double cy = g_winH * 0.5;

      if (!g_ignoreNextWarpEvent) {
        double dx = x - cx;
        double dy = y - cy;

        // clamp spikes
        const double maxDelta = 250.0;
        if (dx >  maxDelta) dx =  maxDelta;
        if (dx < -maxDelta) dx = -maxDelta;
        if (dy >  maxDelta) dy =  maxDelta;
        if (dy < -maxDelta) dy = -maxDelta;

        // deadzone
        const double dead = 0.00005;
        if (std::abs(dx) > dead || std::abs(dy) > dead) {
          // FPS convention: invert Y (optional). If this feels wrong, remove the minus.
          Input::addMouseDelta((float)dx, (float)-dy);
        }
      }

      // warp back to center every frame
      g_ignoreNextWarpEvent = true;
      glfwSetCursorPos(window, cx, cy);
    }

    // After warp, allow next real sample
    if (g_ignoreNextWarpEvent) {
      // Next frame, we will read position after warp; ignore exactly one sample
      g_ignoreNextWarpEvent = false;
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
