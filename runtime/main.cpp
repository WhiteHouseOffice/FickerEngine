#include "core/Engine.h"

#ifdef FE_WEB
  #include <emscripten/emscripten.h>
  static void tick(void*) { Engine::instance().stepOnce(); }
  int main() {
    Engine::instance().init();
    emscripten_set_main_loop_arg(tick, nullptr, 30, 1);
    return 0;
  }
#else
  #include <thread>
  #include <chrono>
  int main() {
    Engine::instance().init();
    for (;;) {
      Engine::instance().stepOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    return 0;
  }
#endif
