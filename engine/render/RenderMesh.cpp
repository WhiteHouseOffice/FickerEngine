#include "render/RenderMesh.h"
#include "render/WebGPUContext.h"
#include <cstring>

using namespace render;

static inline WGPUStringView sv(const char* s) {
  return WGPUStringView{ s, (size_t)std::strlen(s) };
}

void RenderMesh::UploadCPU(const MeshCPU& d)
{
  auto& ctx = WebGPUContext::Get();

  // Vertex buffer
  {
    WGPUBufferDescriptor vb{};
    vb.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
    vb.size  = d.positions.size() * sizeof(float);
    vb.label = sv("grid_vbo");
    vbo = wgpuDeviceCreateBuffer(ctx.device, &vb);
    if (vb.size && vbo) {
      wgpuQueueWriteBuffer(ctx.queue, vbo, 0, d.positions.data(), vb.size);
    }
  }

  // Index buffer
  {
    WGPUBufferDescriptor ib{};
    ib.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
    ib.size  = d.indices.size() * sizeof(uint32_t);
    ib.label = sv("grid_ibo");
    ibo = wgpuDeviceCreateBuffer(ctx.device, &ib);
    if (ib.size && ibo) {
      wgpuQueueWriteBuffer(ctx.queue, ibo, 0, d.indices.data(), ib.size);
    }
  }

  indexCount  = static_cast<uint32_t>(d.indices.size());
  indexFormat = WGPUIndexFormat_Uint32;
  topology    = WGPUPrimitiveTopology_LineList; // grid lines
}

void RenderMesh::Draw(WGPURenderPassEncoder pass) const
{
  if (!vbo || !ibo || indexCount == 0) return;

  wgpuRenderPassEncoderSetVertexBuffer(pass, 0, vbo, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderSetIndexBuffer (pass, ibo, indexFormat, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderDrawIndexed(pass, indexCount, 1, 0, 0, 0);
}
