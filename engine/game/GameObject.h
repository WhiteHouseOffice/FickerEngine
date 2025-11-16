#pragma once

#include "math/MiniMath.h"

class GameObject {
public:
  GameObject() = default;
  virtual ~GameObject() = default;

  // Simple stub: derived classes can override, or we flesh this out later.
  virtual void update(float dt);

  Vec3 position{0.f, 0.f, 0.f};
  float yaw{0.f};
};
