#include "game/Game.h"
#include <algorithm>

void Game::fixedUpdate(double dt) {
  auto& in = Input::instance();

  const float accel = 200.0f; // px/s^2 (arbitrary units for now)
  const float maxv  = 300.0f;

  float ax = 0.f, ay = 0.f;
  if (in.left())  ax -= accel;
  if (in.right()) ax += accel;
  if (in.up())    ay -= accel;
  if (in.down())  ay += accel;

  _p.vx += ax * (float)dt;
  _p.vy += ay * (float)dt;

  // clamp velocity
  _p.vx = std::clamp(_p.vx, -maxv, maxv);
  _p.vy = std::clamp(_p.vy, -maxv, maxv);

  _p.x += _p.vx * (float)dt;
  _p.y += _p.vy * (float)dt;

  // crude friction
  _p.vx *= 0.95f;
  _p.vy *= 0.95f;
}
