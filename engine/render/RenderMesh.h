#pragma once
#include <cstdint>
#include <vector>
#include <webgpu/webgpu.h>

namespace render {

struct MeshCPU {
  std::vector<float>    positions;   // xyz…
  std::vector<uint32_t> indices;     // line/triangle indices
};

// Minimal GPU mesh wrapper for WebGPU
class RenderMesh {
public:
  void UploadCPU(const MeshCPU& d);
  void Draw(WGPURenderPassEncoder pass) const;

  WGPUBuffer vbo = nullptr;
  WGPUBuffer ibo = nullptr;

  uint32_t           indexCount  = 0;
  WGPUIndexFormat    indexFormat = WGPUIndexFormat_Uint32;
  WGPUPrimitiveTopology topology = WGPUPrimitiveTopology_LineList;
};

} // namespace render
