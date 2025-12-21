#include <cstdio>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "core/Engine.h"
#include "core/Input.h"

static void glfw_key_callback(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/) {
  // Only care about press/release (ignore repeat as separate event)
  if (action != GLFW_PRESS && action != GLFW_RELEASE) return;
  const bool down = (action == GLFW_PRESS);

  switch (key) {
    case GLFW_KEY_W: Input::setKey(Input::KEY_W, down); break;
    case GLFW_KEY_A: Input::setKey(Input::KEY_A, down); break;
    case GLFW_KEY_S: Input::setKey(Input::KEY_S, down); break;
    case GLFW_KEY_D: Input::setKey(Input::KEY_D, down); break;

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
  // If focus changes, clear stuck keys/mouse deltas.
  // This fixes “walking forever” when release happens while unfocused.
  (void)focused;
  Input::resetAll();
}

int main() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

  // Force legacy / compatibility GL so fixed-function draw works
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

  // Capture mouse inside the window (no visible cursor)
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // Optional: raw mouse motion if supported (better feel)
  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
  }

  glfwSetKeyCallback(window, glfw_key_callback);
  glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);
  glfwSetWindowFocusCallback(window, glfw_focus_callback);

  glEnable(GL_DEPTH_TEST);

  Engine::instance().init();

  while (!glfwWindowShouldClose(window)) {
    glClearColor(0.1f, 0.2f, 0.35f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Engine::instance().stepOnce();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  Engine::instance().shutdown();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
