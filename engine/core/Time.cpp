#include "core/Time.h"

#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h> // emscripten_get_now()
#else
  #include <chrono>
#endif

namespace {
    double g_last = 0.0;
    double g_dt   = 0.0;

    inline double now_seconds() {
    #ifdef __EMSCRIPTEN__
        // High-resolution timer (milliseconds) → seconds
        return emscripten_get_now() * 0.001;
    #else
        using clock = std::chrono::steady_clock;
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    #endif
    }
}

namespace Time {

void init() {
    g_last = now_seconds();
    g_dt   = 0.0;
}

void update() {
    const double t = now_seconds();
    g_dt   = t - g_last;
    g_last = t;

    // Clamp to sane range (avoids huge dt after tab suspend)
    if (g_dt < 0.0)   g_dt = 0.0;
    if (g_dt > 0.2)   g_dt = 0.2; // cap at ~5 FPS worst case
}

float deltaTime() {
    return static_cast<float>(g_dt);
}

} // namespace Time
