#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
#endif

#include "core/Engine.h"

static void tick(void*) {
  Engine::instance().stepOnce();
}

int main() {
  Engine::instance().init();

#ifdef __EMSCRIPTEN__
  // Drive the engine via the browser loop
  emscripten_set_main_loop_arg(tick, nullptr, 0, true);
#else
  // Simple fallback loop if running natively
  for (;;) Engine::instance().stepOnce();
#endif

  Engine::instance().shutdown();
  return 0;
}
