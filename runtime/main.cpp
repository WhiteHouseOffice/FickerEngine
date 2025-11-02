#include "core/Engine.h"

#if defined(FE_WEB)
#include <emscripten/emscripten.h>
extern "C" {
  // Called by JS (see index.html), after WASM runtime is ready
  EMSCRIPTEN_KEEPALIVE
  void fe_init() { Engine::instance().init(); }
}
int main() {
  // No main loop here; JS calls fe_init() then stepOnce() per frame/capture.
  return 0;
}
#else
// Native stub for completeness
int main() {
  auto& eng = Engine::instance();
  eng.init();
  // Minimal native loop (escape after some steps for now)
  for (int i = 0; i < 600; ++i) { // ~10 seconds at 60Hz
    eng.stepOnce();
    // TODO: call native renderer when available
  }
  return 0;
}
#endif
