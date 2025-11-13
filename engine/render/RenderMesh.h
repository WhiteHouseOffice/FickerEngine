#pragma once

#include <vector>
#include <cstdint>

#if FE_WEBGPU
  #include <webgpu/webgpu.h>
#else
  // Minimal stubs so native builds don't explode if this header is included.
  typedef struct WGPUBufferImpl*            WGPUBuffer;
  typedef struct WGPURenderPassEncoderImpl* WGPURenderPassEncoder;
  typedef std::uint32_t                     WGPUIndexFormat;
  typedef std::uint32_t                     WGPUPrimitiveTopology;
#endif

namespace render {

// CPU-side mesh data coming from geom/ (GridPlane, MarkerCross, etc.)
struct MeshData
{
  std::vector<float>         positions;   // interleaved vertex data (at least xyz)
  std::vector<std::uint16_t> indices;     // 16-bit indices are enough for now

#if FE_WEBGPU
  WGPUIndexFormat         indexFormat = WGPUIndexFormat_Uint16;
  WGPUPrimitiveTopology   topology    = WGPUPrimitiveTopology_TriangleList;
#else
  WGPUIndexFormat         indexFormat = 0;
  WGPUPrimitiveTopology   topology    = 0;
#endif
};

class RenderMesh
{
public:
  RenderMesh()          = default;
  ~RenderMesh()         { Release(); }

  // Upload CPU geometry to GPU buffers (WebGPU path only if FE_WEBGPU=1).
  void UploadCPU(const MeshData& data);

  // Bind buffers and issue a drawIndexed call into an existing render pass.
  void Bind(WGPURenderPassEncoder pass) const;

  // Free GPU buffers (safe to call multiple times).
  void Release();

private:
#if FE_WEBGPU
  WGPUBuffer vbo = nullptr;
  WGPUBuffer ibo = nullptr;
#else
  void*      vbo = nullptr;
  void*      ibo = nullptr;
#endif

  std::uint32_t       indexCount       = 0;
  std::uint64_t       vertexBufferSize = 0;
  std::uint64_t       indexBufferSize  = 0;
  WGPUIndexFormat     indexFormat      = 0;
  WGPUPrimitiveTopology topology       = 0;
};

} // namespace render
