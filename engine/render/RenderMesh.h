#pragma once

#include <vector>
#include "math/MiniMath.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

namespace render {

// Pure CPU mesh data: positions + indices
struct MeshData {
  std::vector<Vec3>        positions;
  std::vector<std::uint32_t> indices;
};

class RenderMesh {
public:
  RenderMesh() = default;

  // Upload purely to CPU-side storage for now (no real GPU)
  void uploadCPU(const MeshData& src);

  const MeshData& data() const { return mesh; }

private:
  MeshData mesh;
};

// Helpers to convert from geometry data to mesh:
MeshData MakeMesh(const geom::GridData& grid);
MeshData MakeMesh(const geom::LinesData& lines);

} // namespace render
