#pragma once

#include "math/MiniMath.h"
#include "render/RenderMesh.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

class Scene {
public:
  void init();
  void update(float dt);

  // Main scene render
  void render(const Mat4& view, const Mat4& proj);

  // Optional debug overlay (unused for now)
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  geom::GridPlane   grid;
  geom::MarkerCross marker;

  RenderMesh gridMesh;
  RenderMesh markerMesh;
};
