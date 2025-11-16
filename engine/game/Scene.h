// engine/game/Scene.h
#pragma once

#include "math/MiniMath.h"
#include "render/RenderMesh.h"

// Minimal scene that owns CPU meshes for a grid and a marker.
// For now this is purely for debugging vertex/index upload.
class Scene {
public:
  Scene();

  // Called every frame from Engine with delta time.
  void update(float dt);

  // These will later do real drawing; right now they're just stubs.
  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  RenderMesh gridMesh;
  RenderMesh markerMesh;
};
