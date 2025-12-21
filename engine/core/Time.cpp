#include "core/Time.h"
#include <chrono>

namespace {
  float  g_deltaTime = 0.0f;
  float  g_time      = 0.0f;
  double g_lastTime  = 0.0;
}

static double nowSeconds() {
  return std::chrono::duration<double>(
    std::chrono::high_resolution_clock::now().time_since_epoch()
  ).count();
}

void Time::init() {
  g_lastTime  = nowSeconds();
  g_deltaTime = 0.0f;
  g_time      = 0.0f;
}

void Time::update() {
  const double now = nowSeconds();
  g_deltaTime = float(now - g_lastTime);
  g_time     += g_deltaTime;
  g_lastTime  = now;
}

float Time::deltaTime() { return g_deltaTime; }
float Time::time()      { return g_time; }
