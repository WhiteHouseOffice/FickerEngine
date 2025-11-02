#pragma once
#include "core/Input.h"

struct Player {
  float x = 0.f, y = 0.f;
  float vx = 0.f, vy = 0.f;
};

class Game {
public:
  void init() {}
  void fixedUpdate(double dt);

  const Player& player() const { return _p; }
private:
  Player _p;
};
