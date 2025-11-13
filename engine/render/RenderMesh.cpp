#include <cstdio>
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"

namespace render {

void RenderMesh::release() {
#if defined(FE_WEBGPU)
  // If we had real GPU buffers, we'd destroy them here.
  vertexBuffer_ = nullptr;
  indexBuffer_  = nullptr;
#endif
  cpu_.positions.clear();
  cpu_.indices.clear();
}

void RenderMesh::uploadCPU(const MeshData& d) {
  cpu_ = d;

#if defined(FE_WEBGPU)
  // In a real backend we would create GPU buffers here and upload cpu_.*
  std::printf("[RenderMesh] (stub) uploadCPU: %zu verts, %zu indices\n",
              cpu_.positions.size() / 3,
              cpu_.indices.size());
#else
  std::printf("[RenderMesh] (stub) uploadCPU (no WebGPU): %zu verts, %zu indices\n",
              cpu_.positions.size() / 3,
              cpu_.indices.size());
#endif
}

void RenderMesh::bind(WGPURenderPassEncoder /*pass*/) const {
  // Stub: nothing to bind yet.
}

// --- Helpers to turn geom::* into MeshData ---

MeshData MakeMesh(const geom::GridPlane& plane) {
  MeshData m;
  m.positions = plane.positions;                // GridPlane.h should expose these
  m.indices   = plane.indices;
  return m;
}

MeshData MakeMesh(const geom::MarkerCross& marker) {
  MeshData m;
  m.positions = marker.positions;
  m.indices   = marker.indices;
  return m;
}

} // namespace render
