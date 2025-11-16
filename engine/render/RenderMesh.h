#pragma once

#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

// CPU-only render mesh for now (no GPU buffers yet).
class RenderMesh {
public:
  std::vector<Vec3>     positions;
  std::vector<uint32_t> indices;

  void clear();

  // Fill from CPU geometry
  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);
};
