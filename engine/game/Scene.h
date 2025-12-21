#pragma once

#include <vector>
#include <cstdint>

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"
#include "math/MiniMath.h"

class Scene {
public:
  Scene();

  void init();
  void update(float dt);

  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

private:
  geom::GridPlane   grid_;
  geom::MarkerCross marker_;

  engine::render::RenderMesh gridMesh_;
  engine::render::RenderMesh markerMesh_;
  engine::render::RenderMesh groundMesh_;

  void buildGroundQuad(float halfSize);
};
