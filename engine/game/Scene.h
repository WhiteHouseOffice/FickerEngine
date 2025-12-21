#pragma once

#include <cstdint>
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"

class Scene {
public:
  Scene() = default;

  void init();
  void render();

private:
  geom::GridPlane grid_;
  geom::MarkerCross marker_;

  engine::render::RenderMesh groundMesh_;
  engine::render::RenderMesh gridMesh_;
  engine::render::RenderMesh markerMesh_;
};
