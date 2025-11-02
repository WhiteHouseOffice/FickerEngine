#pragma once
#include <cstdint>
#include <vector>

#include <webgpu/webgpu.h>

namespace render {

// CPU-side mesh payload the renderer can upload.
struct MeshData {
  // tightly-packed XYZ (float) vertices
  std::vector<float>    positions;
  // 16- or 32-bit triangle indices (we’ll pick format at upload)
  std::vector<uint32_t> indices;
  // vertex stride in floats (e.g. 3 for xyz, 6 if xyz+normals, etc.)
  uint32_t              strideFloats = 3;
};

class RenderMesh {
public:
  RenderMesh() = default;
  ~RenderMesh() { Release(); }

  // Free GPU buffers (safe to call multiple times).
  void Release();

  // Upload (or re-upload) CPU mesh data to GPU buffers.
  void UploadCPU(const MeshData& d);

  // Bind this mesh in a render pass (sets VB/IB and primitive state inputs).
  void Bind(WGPURenderPassEncoder pass) const;

  // Public read-only draw info
  uint32_t               indexCount  = 0;
  WGPUIndexFormat        indexFormat = WGPUIndexFormat_Undefined;
  WGPUPrimitiveTopology  topology    = WGPUPrimitiveTopology_TriangleList;

private:
  WGPUBuffer vbo = nullptr;
  WGPUBuffer ibo = nullptr;
  uint32_t   vertexStrideBytes = 0;
};

} // namespace render
