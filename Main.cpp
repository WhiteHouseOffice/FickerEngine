#include <cstdio>
#include <cmath>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include "core/Engine.h"
#include "core/Input.h"

// ------------------------------------------------------------
// Mouse capture state (WSLg-safe)
// ------------------------------------------------------------
static bool g_mouseCaptured = false;

// window size for center recentering
static int g_winW = 960;
static int g_winH = 540;

// when we warp cursor to center, GLFW will fire a cursor callback;
// ignore the next event to avoid artificial delta.
static bool g_ignoreNextMouseEvent = false;

static void glfw_window_size_callback(GLFWwindow* /*window*/, int w, int h) {
  g_winW = (w > 1) ? w : 1;
  g_winH = (h > 1) ? h : 1;
}

static void setMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;
  Input::resetMouse();

  glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, captured ? GLFW_TRUE : GLFW_FALSE);
  }

  // Recenter immediately on capture (prevents drift on WSLg/Wayland)
  if (captured) {
    const double cx = g_winW * 0.5;
    const double cy = g_winH * 0.5;
    g_ignoreNextMouseEvent = true;
    glfwSetCursorPos(window, cx, cy);
  }
}

// ------------------------------------------------------------
// Callbacks
// ------------------------------------------------------------
static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  const bool down = (action != GLFW_RELEASE);

  // ESC releases mouse capture
  if (key == GLFW_KEY_ESCAPE && down) {
    setMouseCaptured(window, false);
    return;
  }

  // Map movement keys to Input enum
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
  // Left click captures mouse
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    setMouseCaptured(window, true);
  }
}

static void glfw_cursor_pos_callback(GLFWwindow* window, double x, double y) {
  (void)window;
  if (!g_mouseCaptured) return;

  // When we recenter, GLFW emits a synthetic move event. Ignore it.
  if (g_ignoreNextMouseEvent) {
    g_ignoreNextMouseEvent = false;
    return;
  }

  const double cx = g_winW * 0.5;
  const double cy = g_winH * 0.5;

  double dx = x - cx;
  double dy = y - cy;

  // Clamp spikes (WSLg/compositor glitches)
  const double maxDelta = 250.0;
  if (dx >  maxDelta) dx =  maxDelta;
  if (dx < -maxDelta) dx = -maxDelta;
  if (dy >  maxDelta) dy =  maxDelta;
  if (dy < -maxDelta) dy = -maxDelta;

  // Deadzone to prevent tiny drift from accumulating
  const double dead = 0.00005;
  const bool tiny = (std::abs(dx) < dead && std::abs(dy) < dead);

  if (!tiny) {
    Input::addMouseDelta((float)dx, (float)dy);
  }

  // Recenter every event so cursor never "walks" (kills bottom-right drift)
  g_ignoreNextMouseEvent = true;
  glfwSetCursorPos(window, cx, cy);
}

static void glfw_focus_callback(GLFWwindow* window, int focused) {
  if (!focused) {
    setMouseCaptured(window, false);
    Input::resetAll();
  } else {
    // On refocus, avoid a huge delta and recentre if captured later
    g_ignoreNextMouseEvent = false;
    Input::resetMouse();
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

  // Basic OpenGL context (legacy-friendly)
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

  // Callbacks
  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);
  glfwSetWindowSizeCallback(window, glfw_window_size_callback);

  // Init engine
  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

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
