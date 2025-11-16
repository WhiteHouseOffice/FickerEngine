#include "game/Scene.h"

Scene::Scene() = default;

void Scene::buildDefaultScene() {
  // Build CPU geometry
  m_gridCPU   = geom::GridPlane::MakeXZ(10, 1.0f);   // 21x21 grid, 1m spacing
  m_markerCPU = geom::MarkerCross::MakeOrigin(0.5f); // small axis cross

  // Upload into CPU-only RenderMesh containers
  m_gridMesh.clear();
  m_gridMesh.uploadGrid(m_gridCPU);

  m_markerMesh.clear();
  m_markerMesh.uploadMarker(m_markerCPU);
}

void Scene::update(float /*dt*/) {
  // Static scene for now.
  // Later we'll put object animation / edit-mode stuff here.
}
