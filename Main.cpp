#include <cstdio>
#include <cmath>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include "core/Engine.h"
#include "core/Input.h"

// ------------------------------------------------------------
// Mouse capture state
// ------------------------------------------------------------
static bool   g_mouseCaptured = false;
static bool   g_haveLastMouse = false;
static double g_lastMouseX = 0.0;
static double g_lastMouseY = 0.0;

static void setMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;
  g_haveLastMouse = false;   // prevents big first delta
  Input::resetMouse();

  glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  // Helps on some setups (including some WSLg configs)
  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, captured ? GLFW_TRUE : GLFW_FALSE);
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

  // Optional: toggle capture with F
  if (key == GLFW_KEY_F && down) {
    setMouseCaptured(window, !g_mouseCaptured);
    return;
  }

  // Map movement keys to your Input enum
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

    default: break;
  }
}

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
  // Left click captures mouse
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    setMouseCaptured(window, true);
  }
}

static void glfw_cursor_pos_callback(GLFWwindow* /*window*/, double x, double y) {
  if (!g_mouseCaptured) return;

  // First sample after capture/focus: just seed, no delta
  if (!g_haveLastMouse) {
    g_lastMouseX = x;
    g_lastMouseY = y;
    g_haveLastMouse = true;
    return;
  }

  double dx = x - g_lastMouseX;
  double dy = y - g_lastMouseY;

  g_lastMouseX = x;
  g_lastMouseY = y;

  // Clamp insane spikes (WSLg/compositor weirdness)
  const double maxDelta = 200.0;
  if (dx >  maxDelta) dx =  maxDelta;
  if (dx < -maxDelta) dx = -maxDelta;
  if (dy >  maxDelta) dy =  maxDelta;
  if (dy < -maxDelta) dy = -maxDelta;

  // Deadzone to kill tiny drift
  const double dead = 0.00005;
  if (std::abs(dx) < dead && std::abs(dy) < dead) return;

  // ✅ Send true deltas into Input
  Input::addMouseDelta(static_cast<float>(dx), static_cast<float>(dy));
}

static void glfw_focus_callback(GLFWwindow* window, int focused) {
  if (!focused) {
    // On alt-tab: release capture and clear input so keys don't stick
    setMouseCaptured(window, false);
    Input::resetAll();
  } else {
    // On refocus: avoid a huge delta
    g_haveLastMouse = false;
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
