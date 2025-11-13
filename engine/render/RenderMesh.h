#pragma once
#include <cstdint>
#include <vector>

namespace geom {
struct GridPlane;
struct MarkerCross;
} // namespace geom

namespace render {

// Again, fake handle types so we can compile without real WebGPU headers.
#if defined(FE_WEBGPU)
using WGPUBuffer            = void*;
using WGPUIndexFormat       = std::uint32_t;
using WGPUPrimitiveTopology = std::uint32_t;
using WGPURenderPassEncoder = void*;
#else
using WGPURenderPassEncoder = void*;
#endif

// Simple CPU-side mesh data structure used by the editor/game layer.
struct MeshData {
  std::vector<float>         positions;   // xyz triples
  std::vector<std::uint16_t> indices;     // triangle indices
};

// RenderMesh is currently a CPU-only stub. We keep the API surface
// so we can plug a real GPU backend later.
class RenderMesh {
public:
  RenderMesh()  = default;
  ~RenderMesh() { release(); }

  // Free any GPU/CPU resources
  void release();

  // Upload CPU data (currently just copies into cpu_ buffer)
  void uploadCPU(const MeshData& d);

  // Bind to a render pass (no-op for now)
  void bind(WGPURenderPassEncoder pass) const;

  bool valid() const { return !cpu_.positions.empty(); }

private:
  // CPU-side copy of the mesh. In a real backend we'd keep GPU buffers too.
  MeshData cpu_;

#if defined(FE_WEBGPU)
  WGPUBuffer            vertexBuffer_      = nullptr;
  WGPUBuffer            indexBuffer_       = nullptr;
  WGPUIndexFormat       indexFormat_       = 0;
  WGPUPrimitiveTopology topology_          = 0;
#endif
};

// Helpers to build meshes from your geom types:
MeshData MakeMesh(const geom::GridPlane& plane);
MeshData MakeMesh(const geom::MarkerCross& marker);

} // namespace render
