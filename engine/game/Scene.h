#pragma once

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"
#include "math/MiniMath.h"

// Simple scene container:
// - owns CPU geometry (GridPlane / MarkerCross)
// - owns CPU "render meshes" that mirror that geometry
class Scene {
public:
  Scene();

  // One-time setup of geometry and CPU "render meshes".
  void init();

  // Advance any scene-level simulation (currently no-op).
  void update(float dt);

  // Placeholder draw hooks for the future GPU renderer.
  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  geom::GridPlane   grid_;
  geom::MarkerCross marker_;

  RenderMesh gridMesh_;
  RenderMesh markerMesh_;
};
