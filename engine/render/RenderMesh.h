#pragma once

#include <vector>
#include "math/MiniMath.h"

// Forward declarations of geometry types
namespace geom {
struct GridPlane;
struct MarkerCross;
} // namespace geom

namespace render {

// Pure CPU-side mesh data for now
struct MeshData {
  std::vector<Vec3>     positions;
  std::vector<uint32_t> indices;
};

class RenderMesh {
public:
  RenderMesh() = default;
  ~RenderMesh() = default;

  // Upload from CPU geometry generators
  void uploadGrid(const geom::GridPlane& plane);
  void uploadMarker(const geom::MarkerCross& marker);

  // GPU-related hooks – currently no-op in the stub backend
  void release();
  void bind() const;

  const MeshData& data() const { return mesh; }

private:
  MeshData mesh;
};

} // namespace render
