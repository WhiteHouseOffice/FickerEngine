#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include "core/Engine.h"
#include "core/Input.h"

// ------------------------------------------------------------
// Tuning
// ------------------------------------------------------------
static constexpr float MOUSE_SENS = 1.0f;

// If your camera feels inverted, change these (don’t play whack-a-mole elsewhere)
static constexpr bool INVERT_X = false;
static constexpr bool INVERT_Y = true;   // common FPS: mouse up = look up

// ------------------------------------------------------------
// Runtime detection: WSL / WSLg tends to be weird with GLFW_CURSOR_DISABLED
// ------------------------------------------------------------
static bool isWSL() {
  // Common env vars in WSL
  return (std::getenv("WSL_DISTRO_NAME") != nullptr) ||
         (std::getenv("WSL_INTEROP") != nullptr) ||
         (std::getenv("WSLENV") != nullptr);
}

// ------------------------------------------------------------
// Mouse capture state
// ------------------------------------------------------------
static bool g_mouseCaptured = false;

// For disabled-cursor delta mode
static bool   g_haveLast = false;
static double g_lastX = 0.0;
static double g_lastY = 0.0;

// For warp-lock mode
static int  g_winW = 960;
static int  g_winH = 540;
static bool g_skipOnce = true;

static void glfw_window_size_callback(GLFWwindow*, int w, int h) {
  g_winW = (w > 1) ? w : 1;
  g_winH = (h > 1) ? h : 1;
}

static void warpToCenter(GLFWwindow* window) {
  const double cx = g_winW * 0.5;
  const double cy = g_winH * 0.5;
  glfwSetCursorPos(window, cx, cy);
}

// Choose mode:
// - Non-WSL: real lock with GLFW_CURSOR_DISABLED
// - WSL: warp-lock with HIDDEN cursor (feels locked, avoids biased disabled-mode)
static bool useWarpLock(GLFWwindow*) {
  return isWSL();
}

static void setMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;

  Input::resetMouse();
  g_haveLast = false;
  g_skipOnce = true;

  if (!captured) {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    if (glfwRawMouseMotionSupported())
      glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
    return;
  }

  if (!useWarpLock(window)) {
    // ✅ REAL LOCK (X11): confine cursor to window
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // raw motion helps when available
    if (glfwRawMouseMotionSupported())
      glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

    // seed "last"
    double x=0,y=0;
    glfwGetCursorPos(window, &x, &y);
    g_lastX = x;
    g_lastY = y;
    g_haveLast = true;
  } else {
    // ✅ WSLg fallback "lock": hide cursor and warp to center every frame
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
    warpToCenter(window);
  }
}

// ------------------------------------------------------------
// Input callbacks
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
    g_haveLast = false;
    g_skipOnce = true;
    if (g_mouseCaptured && useWarpLock(window)) {
      warpToCenter(window);
    }
  }
}

// We still keep this callback; in disabled mode it’s the best source of deltas.
// In warp mode, we ignore it and compute deltas per-frame in the loop.
static void glfw_cursor_pos_callback(GLFWwindow*, double x, double y) {
  if (!g_mouseCaptured) return;
  if (useWarpLock(nullptr)) return;

  if (!g_haveLast) {
    g_lastX = x; g_lastY = y;
    g_haveLast = true;
    return;
  }

  double dx = x - g_lastX;
  double dy = y - g_lastY;
  g_lastX = x; g_lastY = y;

  // clamp spikes
  const double maxDelta = 300.0;
  if (dx >  maxDelta) dx =  maxDelta;
  if (dx < -maxDelta) dx = -maxDelta;
  if (dy >  maxDelta) dy =  maxDelta;
  if (dy < -maxDelta) dy = -maxDelta;

  // deadzone
  const double dead = 0.00005;
  if (std::abs(dx) < dead && std::abs(dy) < dead) return;

  float fx = (float)dx * MOUSE_SENS;
  float fy = (float)dy * MOUSE_SENS;
  if (INVERT_X) fx = -fx;
  if (INVERT_Y) fy = -fy;

  Input::addMouseDelta(fx, fy);
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
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);

  std::printf("[mouse] mode = %s\n", useWarpLock(window) ? "warp-lock (WSL fallback)" : "cursor-disabled (real lock)");

  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Warp-lock mode: compute deltas from center and warp back every frame
    if (g_mouseCaptured && useWarpLock(window)) {
      double x=0.0, y=0.0;
      glfwGetCursorPos(window, &x, &y);

      const double cx = g_winW * 0.5;
      const double cy = g_winH * 0.5;

      double dx = x - cx;
      double dy = y - cy;

      warpToCenter(window);

      if (g_skipOnce) {
        g_skipOnce = false;
      } else {
        const double maxDelta = 300.0;
        if (dx >  maxDelta) dx =  maxDelta;
        if (dx < -maxDelta) dx = -maxDelta;
        if (dy >  maxDelta) dy =  maxDelta;
        if (dy < -maxDelta) dy = -maxDelta;

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
