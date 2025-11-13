#include "render/RenderMesh.h"

#if FE_WEBGPU
  #include "render/WebGPUContext.h"
#endif

namespace render
{

void RenderMesh::Release()
{
#if FE_WEBGPU
  if (vbo)
  {
    wgpuBufferDestroy(vbo);
    wgpuBufferRelease(vbo);
    vbo = nullptr;
  }
  if (ibo)
  {
    wgpuBufferDestroy(ibo);
    wgpuBufferRelease(ibo);
    ibo = nullptr;
  }
#endif

  indexCount       = 0;
  vertexBufferSize = 0;
  indexBufferSize  = 0;
}

void RenderMesh::UploadCPU(const MeshData& data)
{
  // Drop any previous GPU buffers
  Release();

#if FE_WEBGPU
  auto& ctx = WebGPUContext::Get();
  if (!ctx.Device() || !ctx.Queue())
  {
    // WebGPU not ready yet – quietly skip upload.
    return;
  }

  // --- Vertex buffer ---
  if (!data.positions.empty())
  {
    vertexBufferSize = static_cast<std::uint64_t>(data.positions.size() * sizeof(float));

    WGPUBufferDescriptor vb{};
    vb.usage           = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
    vb.size            = vertexBufferSize;
    vb.mappedAtCreation = false;   // we upload via QueueWriteBuffer

    vbo = wgpuDeviceCreateBuffer(ctx.Device(), &vb);
    if (vbo)
    {
      wgpuQueueWriteBuffer(ctx.Queue(), vbo, 0, data.positions.data(), vertexBufferSize);
    }
  }

  // --- Index buffer ---
  if (!data.indices.empty())
  {
    indexBufferSize = static_cast<std::uint64_t>(data.indices.size() * sizeof(std::uint16_t));

    WGPUBufferDescriptor ib{};
    ib.usage           = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
    ib.size            = indexBufferSize;
    ib.mappedAtCreation = false;

    ibo = wgpuDeviceCreateBuffer(ctx.Device(), &ib);
    if (ibo)
    {
      wgpuQueueWriteBuffer(ctx.Queue(), ibo, 0, data.indices.data(), indexBufferSize);
    }

    indexCount  = static_cast<std::uint32_t>(data.indices.size());
    indexFormat = data.indexFormat;
    topology    = data.topology;
  }
#else
  (void)data; // avoid unused warnings on native builds
#endif
}

void RenderMesh::Bind(WGPURenderPassEncoder pass) const
{
#if FE_WEBGPU
  if (!pass || !vbo || !ibo || indexCount == 0)
    return;

  // Bind vertex/index buffers and draw.
  wgpuRenderPassEncoderSetVertexBuffer(
      pass,
      0,              // slot
      vbo,
      0,              // offset
      vertexBufferSize);

  wgpuRenderPassEncoderSetIndexBuffer(
      pass,
      ibo,
      indexFormat,
      0,              // offset
      indexBufferSize);

  wgpuRenderPassEncoderDrawIndexed(
      pass,
      indexCount,
      1,              // instanceCount
      0,              // firstIndex
      0,              // baseVertex
      0);             // firstInstance
#else
  (void)pass;
#endif
}

} // namespace render
