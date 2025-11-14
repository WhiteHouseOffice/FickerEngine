#pragma once

#include "math/MiniMath.h"

// Very small camera/game controller for now.
class Game {
public:
  void init();
  void update(float dt);
  Mat4 view() const;

  const Vec3& cameraPosition() const { return cameraPos; }

private:
  Vec3  cameraPos{0.0f, 1.5f, 5.0f};
  float yaw       = 0.0f;   // radians, turn left/right
  float moveSpeed = 4.0f;   // units per second
};
