#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include "core/Engine.h"
#include "core/Input.h"

// ------------------------------------------------------------
// Tuning (change these, not random signs elsewhere)
// ------------------------------------------------------------
static constexpr float MOUSE_SENS = 0.25f;  // lower = less sensitive
static constexpr bool  INVERT_X   = false;  // set true if left/right feels inverted
static constexpr bool  INVERT_Y   = true;   // common FPS: mouse up = look up

// ------------------------------------------------------------
// Mouse capture state
// ------------------------------------------------------------
static bool g_mouseCaptured = false;
static int  g_winW = 960;
static int  g_winH = 540;
static bool g_skipDeltaOnce = true;

static void glfw_window_size_callback(GLFWwindow*, int w, int h) {
  g_winW = (w > 1) ? w : 1;
  g_winH = (h > 1) ? h : 1;
}

static void warpToCenter(GLFWwindow* window) {
  const double cx = g_winW * 0.5;
  const double cy = g_winH * 0.5;
  glfwSetCursorPos(window, cx, cy);
}

static void setMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;
  Input::resetMouse();
  g_skipDeltaOnce = true;

  // Hide cursor while captured (we manually lock by warping to center)
  glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_HIDDEN : GLFW_CURSOR_NORMAL);

  if (captured) {
    warpToCenter(window);
  }
}

// ------------------------------------------------------------
// Callbacks
// ------------------------------------------------------------
static void glfw_key_callback(GLFWwindow* window, int key, int, int action, int) {
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

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int) {
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
    g_skipDeltaOnce = true;
    if (g_mouseCaptured) warpToCenter(window);
  }
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------
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

    // --------------------------------------------------------
    // ✅ Teleport-to-center mouse lock (EVERY FRAME)
    // --------------------------------------------------------
    if (g_mouseCaptured) {
      double x = 0.0, y = 0.0;
      glfwGetCursorPos(window, &x, &y);

      const double cx = g_winW * 0.5;
      const double cy = g_winH * 0.5;

      double dx = x - cx;
      double dy = y - cy;

      // Immediately lock back to center
      warpToCenter(window);

      if (g_skipDeltaOnce) {
        g_skipDeltaOnce = false;
      } else {
        // Clamp spikes
        const double maxDelta = 250.0;
        if (dx >  maxDelta) dx =  maxDelta;
        if (dx < -maxDelta) dx = -maxDelta;
        if (dy >  maxDelta) dy =  maxDelta;
        if (dy < -maxDelta) dy = -maxDelta;

        // Deadzone
        const double dead = 0.00005;
        if (std::abs(dx) > dead || std::abs(dy) > dead) {
          float fx = (float)dx * MOUSE_SENS;
          float fy = (float)dy * MOUSE_SENS;

          if (INVERT_X) fx = -fx;
          if (INVERT_Y) fy = -fy;

          Input::addMouseDelta(fx, fy);
        }
      }
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
