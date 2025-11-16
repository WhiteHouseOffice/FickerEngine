#include "game/Scene.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"

Scene::Scene() {
  geom::GridPlane g;
  g.build(10.0f, 10);
  gridMesh.uploadGrid(g);

  geom::MarkerCross m;
  m.build(0.5f);
  markerMesh.uploadMarker(m);
}

void Scene::update(float dt) {
  // nothing yet
}

void Scene::render(const Mat4& view, const Mat4& proj) {
  gridMesh.debugPrint("grid");
  markerMesh.debugPrint("marker");
}

void Scene::renderDebug(const Mat4& view, const Mat4& proj) {
  render(view, proj);
}
