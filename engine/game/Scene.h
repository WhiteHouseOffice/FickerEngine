#pragma once

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"

// Simple scene: a grid floor and an origin marker
class Scene {
public:
  Scene();

  void buildDefaultScene();  // create CPU geometry and upload into meshes
  void update(float dt);     // for now: no-op, but keep API

  const RenderMesh& gridMesh()   const { return m_gridMesh; }
  const RenderMesh& markerMesh() const { return m_markerMesh; }

private:
  geom::GridPlane   m_gridCPU;
  geom::MarkerCross m_markerCPU;

  RenderMesh m_gridMesh;
  RenderMesh m_markerMesh;
};
