#include "engine/core/Time.h"
#include <chrono>
#include <algorithm>

static double now_seconds() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

Time::Time() : _last(now_seconds()) {}

double Time::tick() {
  const double t = now_seconds();
  double dt = t - _last;
  _last = t;
  // Clamp dt to avoid spiral of death if tab was suspended
  dt = std::clamp(dt, 0.0, 0.25);
  return dt;
}
