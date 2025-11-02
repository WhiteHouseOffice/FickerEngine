#include "core/Engine.h"

#ifdef FE_WEB
  #include <emscripten/emscripten.h>
  static void tick(void*) { Engine::instance().stepOnce(); }
  int main() {
    Engine::instance().init();
    // Let the browser drive ~60 FPS; simulateInfiniteLoop=1 keeps main from returning.
    emscripten_set_main_loop_arg(tick, nullptr, 0, 1);
    return 0;
  }
#else
  #include <thread>
  #include <chrono>
  int main() {
    Engine::instance().init();
    for (;;) {
      Engine::instance().stepOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
  }
#endif
