#include "core/Input.h"

#ifdef __EMSCRIPTEN__
  #include <emscripten/html5.h>
  #include <cstring>
#endif

namespace {
  bool g_keys[Input::KEY_COUNT] = {};
}

#ifdef __EMSCRIPTEN__
// Common callback for keydown / keyup
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

  return EM_TRUE;
}
#endif

void Input::init() {
#ifdef __EMSCRIPTEN__
  // Listen on the window so we get keys regardless of focus quirks.
  emscripten_set_keydown_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                  nullptr,
                                  EM_TRUE,
                                  key_callback);
  emscripten_set_keyup_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                nullptr,
                                EM_TRUE,
                                key_callback);
#endif
}

bool Input::isKeyDown(Key key) {
  if (key < 0 || key >= KEY_COUNT) return false;
  return g_keys[key];
}
