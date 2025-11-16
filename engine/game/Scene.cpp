#include "game/Scene.h"
#include <cstdio>

Scene::Scene() {
  // CPU geometry is created by default constructors.
  // If your GridPlane/MarkerCross take parameters, adjust here.
  gridMesh.uploadGrid(grid);
  markerMesh.uploadMarker(originMarker);
}

void Scene::update(float /*dt*/) {
  // Nothing animated yet.
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // TODO: hook CPU RenderMesh up to GPU/WebGPU/WebGL later.
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // TODO: debug rendering will go here once GPU renderer is wired.
}

