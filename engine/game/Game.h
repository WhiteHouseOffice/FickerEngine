#pragma once

#include "math/MiniMath.h"   // Vec3, Mat4, lookAt, perspective

class Game {
public:
  void init();
  void update(float dt);

  Mat4 view() const;
  Mat4 proj(float aspect) const;

  const Vec3& cameraPosition() const { return camPos; }

private:
  Vec3  camPos { 0.f, 2.f, 5.f };
  float yaw       = 0.f;
  float pitch     = 0.f;
  float moveSpeed = 5.f;
  float logTimer  = 0.f;
};
