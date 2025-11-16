#pragma once

#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

// CPU-only render mesh for now.
// Just holds positions + indices; later we'll feed this to WebGPU/WebGL.
class RenderMesh {
public:
  std::vector<Vec3>     positions;
  std::vector<uint32_t> indices;

  void clear();

  // Upload pure CPU geometry into this mesh
  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);
};
