#pragma once

#include "math/MiniMath.h"

class Scene {
public:
  void init();
  void update(float dt);

  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);
};
