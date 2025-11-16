#include "game/Scene.h"

Scene::Scene() = default;

void Scene::init() {
  // Copy CPU geometry into CPU-side "render meshes" once.
  // (If GridPlane / MarkerCross currently don't build any vertices,
  //  you'll still see 0 counts here; that's a separate step later.)
  gridMesh_.uploadGrid(grid_);
  markerMesh_.uploadMarker(marker_);

  // One-time debug dump so we see counts but avoid per-frame spam.
  gridMesh_.debugPrint("grid");
  markerMesh_.debugPrint("marker");
}

void Scene::update(float /*dt*/) {
  // No scene-level simulation yet.
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // No GPU rendering yet; WebGPU/WebGL calls will live here later.
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // No per-frame logging here – keeps the console clean.
}
