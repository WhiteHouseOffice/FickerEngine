#pragma once

#include "math/MiniMath.h"

class Game {
public:
  void init();
  void update(float dt);

  // Camera view matrix
  Mat4 view() const;

  const Vec3& cameraPosition() const { return camPos; }

private:
  Vec3  camPos     { 0.0f, 2.0f, 5.0f };
  Vec3  camForward { 0.0f, 0.0f, -1.0f };
  float moveSpeed  = 4.0f;
  float logTimer   = 0.0f;
};
