#pragma once

#include "math/MiniMath.h"

class Scene {
public:
  virtual ~Scene() = default;

  virtual void init();
  virtual void update(float dt);
  virtual void render(const Mat4& view, const Mat4& proj);
  virtual void renderDebug(const Mat4& view, const Mat4& proj);
};
