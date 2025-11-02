#include "core/Engine.h"

#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
#endif

static Engine g_engine;

static void tick(void*) {
    g_engine.update();
    g_engine.render();
}

int main(int /*argc*/, char** /*argv*/) {
    g_engine.init();

#ifdef __EMSCRIPTEN__
    // 0 = browser-driven FPS, 1 = simulate infinite loop
    emscripten_set_main_loop_arg(tick, nullptr, 0, 1);
#else
    for (;;) { tick(nullptr); }
#endif

    return 0;
}
