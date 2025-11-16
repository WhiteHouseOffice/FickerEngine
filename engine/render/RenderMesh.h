#pragma once

#include <vector>
#include <cstdint>

#include "math/MiniMath.h"

namespace geom {
struct GridPlane;
struct MarkerCross;
}

// CPU-only mesh container for now.
// Later this will know how to upload to WebGPU / WebGL.
class RenderMesh {
public:
  std::vector<Vec3>          positions; // world-space or object-space vertices
  std::vector<std::uint32_t> indices;   // triangle / line indices

  // Upload CPU geometry into this mesh (no GPU work yet).
  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);

  // Step 2: simple debug helper to print mesh stats.
  void debugPrint(const char* label) const;
};
