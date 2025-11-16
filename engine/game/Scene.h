#pragma once

#include "math/MiniMath.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"

class Scene {
public:
  Scene();

  // Advance any scene logic (currently nothing to do).
  void update(float dt);

  // Main scene render – currently a no-op while RenderMesh is CPU-only.
  void render(const Mat4& view, const Mat4& proj);

  // Optional debug render – also a no-op for now.
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  geom::GridPlane   grid;
  geom::MarkerCross marker;

  // CPU-only render meshes; kept around so wiring the future renderer is easy.
  RenderMesh gridMesh;
  RenderMesh markerMesh;
};
