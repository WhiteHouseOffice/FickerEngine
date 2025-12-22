#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include "core/Engine.h"
#include "core/Input.h"

// ------------------------------------------------------------
// Mouse capture state
// ------------------------------------------------------------
static bool   g_mouseCaptured = false;

// WSLg/Wayland can report weird values in disabled cursor mode.
// We guard against bogus streams by requiring "reasonable" deltas.
static double g_lastMouseX = 0.0;
static double g_lastMouseY = 0.0;
static bool   g_haveLastMouse = false;

static void setMouseCaptured(GLFWwindow* window, bool captured) {
  g_mouseCaptured = captured;
  g_haveLastMouse = false;        // IMPORTANT: avoid huge first delta
  Input::resetMouse();            // clear accumulated deltas

  glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  // Try raw mouse motion if available (helps on some stacks)
  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, captured ? GLFW_TRUE : GLFW_FALSE);
  }
}

// ------------------------------------------------------------
// Callbacks
// ------------------------------------------------------------
static void glfw_key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  const bool down = (action != GLFW_RELEASE);

  // Escape always releases mouse + allows window close behavior to work normally
  if (key == GLFW_KEY_ESCAPE && down) {
    setMouseCaptured(window, false);
    return;
  }

  // Optional: click-to-capture toggle key (F)
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

  // Let window close button / alt+F4 etc work
  if (key == GLFW_KEY_Q && down) {
    // optional quit hotkey
    // glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
  // Click inside window to capture mouse
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    setMouseCaptured(window, true);
  }
}

static void glfw_cursor_pos_callback(GLFWwindow* /*window*/, double x, double y) {
  if (!g_mouseCaptured) return;

  // Guard against bogus (0,0) streams that happen on some WSLg/Wayland setups
  if (x == 0.0 && y == 0.0) {
    g_haveLastMouse = false;
    return;
  }

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

  // Clamp insane deltas (prevents runaway spin if compositor feeds garbage)
  const double maxDelta = 120.0;
  if (dx >  maxDelta) dx =  maxDelta;
  if (dx < -maxDelta) dx = -maxDelta;
  if (dy >  maxDelta) dy =  maxDelta;
  if (dy < -maxDelta) dy = -maxDelta;

  // If the stream is still biased (WSLg), it often shows as tiny constant drift.
  // Ignore very small deltas to prevent slow crawling spin.
  const double dead = 0.0001;
  if (std::abs(dx) < dead && std::abs(dy) < dead) return;

  // Feed ONLY deltas into Input
  Input::onMouseMove(g_lastMouseX, g_lastMouseY); // keep internal last consistent
  // But our Input::onMouseMove expects absolute positions; we already updated last above.
  // So instead, we inject the delta via a tiny trick: call onMouseMove with "synthetic" absolute.
  // If your Input is absolute-delta based, use the direct delta injector instead.

  // ✅ Better: if you have no delta injector, use this:
  // We emulate absolute movement by advancing a synthetic cursor position.
  static double sx = 0.0, sy = 0.0;
  sx += dx;
  sy += dy;
  Input::onMouseMove(sx, sy);
}

static void glfw_focus_callback(GLFWwindow* window, int focused) {
  if (!focused) {
    // On alt-tab, release capture and clear input so keys don't "stick"
    setMouseCaptured(window, false);
    Input::resetAll();
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

  // Request a basic OpenGL context
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

  // Init engine
  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Clear screen (your engine currently renders blue anyway)
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
