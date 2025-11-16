#include "game/Scene.h"

Scene::Scene() = default;

void Scene::update(float /*dt*/) {
  // No scene animation or logic yet.
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // TODO: Once we have a GPU renderer:
  // - Feed gridMesh / markerMesh into WebGPU/WebGL draw calls here.
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // TODO: Add any debug-only rendering here (bounds, helpers, etc.).
}
