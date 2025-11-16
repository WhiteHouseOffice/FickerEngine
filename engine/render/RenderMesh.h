#pragma once

#include <vector>

// Geometry lives in engine/geom/, in the global namespace.
struct GridPlane;
struct MarkerCross;

namespace render {

// Simple CPU-side mesh container.
// No WebGPU / GL types here – just raw vertex/index data.
class RenderMesh {
public:
  // 3 floats per vertex: x, y, z
  std::vector<float>        positions;
  std::vector<unsigned int> indices;

  void clear();

  // Upload from CPU geometry generators
  void uploadGrid(const GridPlane& grid);
  void uploadMarker(const MarkerCross& marker);
};

} // namespace render
