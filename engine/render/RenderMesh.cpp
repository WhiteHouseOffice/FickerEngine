#include "render/RenderMesh.h"
#include "render/WebGPUContext.h"
#include <cstring>

using namespace render;

static inline WGPUStringView SV(const char* s) {
  WGPUStringView v{};
  v.data   = s;
  v.length = std::strlen(s);
  return v;
}

void RenderMesh::UploadCPU(const MeshCPU& d)
{
  auto& ctx = WebGPUContext::Get();

  // Vertex buffer
  {
    WGPUBufferDescriptor vb{};
    vb.label = SV("grid_vbo");
    vb.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
    vb.size  = d.positions.size() * sizeof(float);

    vbo = wgpuDeviceCreateBuffer(ctx.Device(), &vb);
    if (vb.size && d.positions.data()) {
      wgpuQueueWriteBuffer(ctx.Queue(), vbo, 0, d.positions.data(), vb.size);
    }
  }

  // Index buffer
  {
    WGPUBufferDescriptor ib{};
    ib.label = SV("grid_ibo");
    ib.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
    ib.size  = d.indices.size() * sizeof(uint32_t);

    ibo = wgpuDeviceCreateBuffer(ctx.Device(), &ib);
    if (ib.size && d.indices.data()) {
      wgpuQueueWriteBuffer(ctx.Queue(), ibo, 0, d.indices.data(), ib.size);
    }
  }

  indexCount  = static_cast<uint32_t>(d.indices.size());
  indexFormat = WGPUIndexFormat_Uint32;
  topology    = WGPUPrimitiveTopology_TriangleList;
}
