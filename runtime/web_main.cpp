#ifdef FE_WEB
#include "core/Engine.h"

// Minimal entry for Emscripten builds.
// Initializes your engine once; stepping is driven from JS (shell.html).
int main() {
    Engine::instance().init();
    return 0;
}
#endif