#pragma once

#include "math/MiniMath.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"

// Very small scene container for now:
// - owns CPU geom (GridPlane, MarkerCross)
// - owns CPU RenderMesh wrappers
// - exposes init/update/render hooks for the Engine
class Scene {
public:
  Scene() = default;  // trivial constructor, no out-of-class definition

  void init();
  void update(float dt);

  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  geom::GridPlane   grid;
  geom::MarkerCross marker;

  RenderMesh gridMesh;
  RenderMesh markerMesh;
};
