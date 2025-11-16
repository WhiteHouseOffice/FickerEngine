#pragma once

#include "math/MiniMath.h"
#include "render/RenderMesh.h"

// Minimal test scene for FickerEngine.
// Holds a world grid and an origin marker as CPU meshes.
class Scene {
public:
  Scene();

  // Build CPU geometry (grid + marker) and upload into RenderMesh.
  void init();

  // Per-frame update hook (currently unused).
  void update(float dt);

  // Render hooks – currently stubs, will be wired when we have a GPU backend.
  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  RenderMesh gridMesh;
  RenderMesh markerMesh;
};
