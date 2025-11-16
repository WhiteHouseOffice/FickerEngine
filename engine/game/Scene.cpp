#include "game/Scene.h"

Scene::Scene() {
  // Build CPU geometry once.
  // Tweak these parameters to taste.
  grid_.build(/*size*/ 10.0f, /*subdivisions*/ 20);
  marker_.build(/*size*/ 0.5f);
}

void Scene::init() {
  // Copy CPU geometry into CPU-side "render meshes" once.
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
