#pragma once

#include "math/MiniMath.h"   // Mat4, Vec3 etc (whatever your engine uses)

class Scene {
public:
  void init();
  Scene() = default;

  // Engine expects these:
  void update(float dt);
  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

  // Optional compatibility wrapper (so other code calling render() still works)
  void render() { render(Mat4::identity(), Mat4::identity()); }
};
