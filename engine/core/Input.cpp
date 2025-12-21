#include "core/Input.h"

#ifdef __EMSCRIPTEN__
  #include <emscripten/html5.h>
  #include <cstring>
#endif

namespace {
  bool g_keys[Input::KEY_COUNT] = {};
  bool g_mouseButtons[Input::MOUSE_BUTTON_COUNT] = {};
  float g_mouseDeltaX = 0.0f;
  float g_mouseDeltaY = 0.0f;
}

#ifdef __EMSCRIPTEN__

static EM_BOOL key_callback(int eventType,
                            const EmscriptenKeyboardEvent* e,
                            void* /*userData*/) {
  using namespace Input;

  const bool isDown = (eventType == EMSCRIPTEN_EVENT_KEYDOWN);

  auto setKey = [isDown, e](const char* keyName, Key key) {
    if (std::strcmp(e->key, keyName) == 0) {
      g_keys[key] = isDown;
    }
  };

  setKey("w",     KEY_W);
  setKey("W",     KEY_W);
  setKey("a",     KEY_A);
  setKey("A",     KEY_A);
  setKey("s",     KEY_S);
  setKey("S",     KEY_S);
  setKey("d",     KEY_D);
  setKey("D",     KEY_D);
  setKey("Shift", KEY_SHIFT);
  setKey(" ",        KEY_SPACE);
  setKey("Spacebar", KEY_SPACE);
  setKey("Control",  KEY_CTRL);

  return EM_TRUE;
}

static EM_BOOL mousemove_callback(int /*eventType*/,
                                  const EmscriptenMouseEvent* e,
                                  void* /*userData*/) {
  g_mouseDeltaX += static_cast<float>(e->movementX);
  g_mouseDeltaY += static_cast<float>(e->movementY);
  return EM_TRUE;
}

static EM_BOOL mousedown_callback(int /*eventType*/,
                                  const EmscriptenMouseEvent* e,
                                  void* /*userData*/) {
  using namespace Input;
  if (e->button == 0) g_mouseButtons[MOUSE_LEFT]   = true;
  if (e->button == 1) g_mouseButtons[MOUSE_MIDDLE] = true;
  if (e->button == 2) g_mouseButtons[MOUSE_RIGHT]  = true;
  return EM_TRUE;
}

static EM_BOOL mouseup_callback(int /*eventType*/,
                                const EmscriptenMouseEvent* e,
                                void* /*userData*/) {
  using namespace Input;
  if (e->button == 0) g_mouseButtons[MOUSE_LEFT]   = false;
  if (e->button == 1) g_mouseButtons[MOUSE_MIDDLE] = false;
  if (e->button == 2) g_mouseButtons[MOUSE_RIGHT]  = false;
  return EM_TRUE;
}

#endif // __EMSCRIPTEN__

void Input::init() {
  // Always reset shared input state.
  #ifdef __EMSCRIPTEN__
    std::memset(g_keys, 0, sizeof(g_keys));
    std::memset(g_mouseButtons, 0, sizeof(g_mouseButtons));
  #else
    for (int i = 0; i < Input::KEY_COUNT; ++i) g_keys[i] = false;
    for (int i = 0; i < Input::MOUSE_BUTTON_COUNT; ++i) g_mouseButtons[i] = false;
  #endif
  g_mouseDeltaX = g_mouseDeltaY = 0.0f;

  #ifdef __EMSCRIPTEN__
    // Keyboard
    emscripten_set_keydown_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                    nullptr,
                                    EM_TRUE,
                                    key_callback);
    emscripten_set_keyup_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                  nullptr,
                                  EM_TRUE,
                                  key_callback);

    // Mouse movement + buttons
    emscripten_set_mousemove_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                      nullptr,
                                      EM_TRUE,
                                      mousemove_callback);
    emscripten_set_mousedown_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                      nullptr,
                                      EM_TRUE,
                                      mousedown_callback);
    emscripten_set_mouseup_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                    nullptr,
                                    EM_TRUE,
                                    mouseup_callback);
  #endif
}

void Input::setKeyDown(Key key, bool down) {
  if (key < 0 || key >= KEY_COUNT) return;
  g_keys[key] = down;
}

void Input::setMouseButtonDown(MouseButton button, bool down) {
  if (button < 0 || button >= MOUSE_BUTTON_COUNT) return;
  g_mouseButtons[button] = down;
}

void Input::addMouseDelta(float dx, float dy) {
  g_mouseDeltaX += dx;
  g_mouseDeltaY += dy;
}

bool Input::isKeyDown(Key key) {
  if (key < 0 || key >= KEY_COUNT) return false;
  return g_keys[key];
}

bool Input::isMouseButtonDown(MouseButton button) {
  if (button < 0 || button >= MOUSE_BUTTON_COUNT) return false;
  return g_mouseButtons[button];
}

void Input::getMouseDelta(float& dx, float& dy) {
  dx = g_mouseDeltaX;
  dy = g_mouseDeltaY;
  g_mouseDeltaX = 0.0f;
  g_mouseDeltaY = 0.0f;
}
