#include "core/Time.h"

#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
#else
  #include <chrono>
#endif

namespace {
  float  g_deltaTime = 0.0f;
  float  g_time      = 0.0f;
  double g_lastTime  = 0.0;
}

void Time::init() {
#ifdef __EMSCRIPTEN__
  g_lastTime = emscripten_get_now() / 1000.0;
#else
  g_lastTime = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()
  ).count();
#endif
  g_deltaTime = 0.0f;
  g_time      = 0.0f;
}

void Time::update() {
#ifdef __EMSCRIPTEN__
  const double now = emscripten_get_now() / 1000.0;
#else
  const double now = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()
  ).count();
#endif

  g_deltaTime = float(now - g_lastTime);
  g_time     += g_deltaTime;
  g_lastTime  = now;
}

float Time::deltaTime() {
  return g_deltaTime;
}

float Time::time() {
  return g_time;
}
