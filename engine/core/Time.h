#pragma once
#include <cstdint>

class Time {
public:
  Time();

  // seconds since last call; clamps to a sane max to avoid huge jumps
  double tick();

  // fixed simulation step (seconds)
  static constexpr double kFixedDt = 1.0 / 60.0;

private:
  double _last; // seconds
};
