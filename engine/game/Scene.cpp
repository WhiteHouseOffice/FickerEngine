#include "game/Scene.h"

#include <cstdio>

void Scene::init() {
  // Build CPU geometry.
  grid.build(10.0f, 10);   // size, subdivisions
  marker.build(0.5f);      // marker "radius"

  // Upload into our CPU-only RenderMeshes.
  gridMesh.uploadGrid(grid);
  markerMesh.uploadMarker(marker);

  // One-time debug print to confirm geometry sizes.
  gridMesh.debugPrint("grid");
  markerMesh.debugPrint("marker");
}

void Scene::update(float /*dt*/) {
  // No per-frame scene logic yet.
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // Placeholder: we'll hook this up once we have a real renderer.
  // For now, keeping this empty avoids unused-parameter warnings.
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // Later this will draw debug overlays. For now we could spam,
  // but that would be noisy, so we keep it quiet.
}
