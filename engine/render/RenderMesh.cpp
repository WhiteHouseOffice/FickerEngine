#include "RenderMesh.h"
#include "WebGPUContext.h"

#include <cstring> // std::strlen

namespace render {

static inline WGPUStringView sv(const char* s) {
  WGPUStringView v;
  v.data = s;
  v.length = static_cast<size_t>(std::strlen(s));
  return v;
}

void RenderMesh::Release() {
  if (vbo) { wgpuBufferRelease(vbo); vbo = nullptr; }
  if (ibo) { wgpuBufferRelease(ibo); ibo = nullptr; }
  indexCount        = 0;
  indexFormat       = WGPUIndexFormat_Undefined;
  topology          = WGPUPrimitiveTopology_TriangleList;
  vertexStrideBytes = 0;
}

void RenderMesh::UploadCPU(const MeshData& d) {
  auto& ctx   = WebGPUContext::Get();
  WGPUDevice dev = ctx.GetDevice();
  WGPUQueue  que = ctx.GetQueue();

  // Decide index format based on data range.
  bool fitsU16 = true;
  for (uint32_t idx : d.indices) {
    if (idx > 0xFFFFu) { fitsU16 = false; break; }
  }
  indexFormat = fitsU16 ? WGPUIndexFormat_Uint16 : WGPUIndexFormat_Uint32;

  // Create / resize VBO
  {
    if (vbo) { wgpuBufferRelease(vbo); vbo = nullptr; }

    WGPUBufferDescriptor vb{};
    vb.nextInChain      = nullptr;
    vb.label            = sv("mesh_vbo");
    vb.usage            = WGPUBufferUsage_CopyDst | WGPUBufferUsage_Vertex;
    vb.size             = static_cast<uint64_t>(d.positions.size() * sizeof(float));
    vb.mappedAtCreation = false;

    vbo = wgpuDeviceCreateBuffer(dev, &vb);
    if (vb.size && !d.positions.empty()) {
      wgpuQueueWriteBuffer(que, vbo, 0, d.positions.data(), static_cast<size_t>(vb.size));
    }
  }

  // Create / resize IBO
  {
    if (ibo) { wgpuBufferRelease(ibo); ibo = nullptr; }

    const uint64_t elemSize = (indexFormat == WGPUIndexFormat_Uint16) ? 2u : 4u;
    WGPUBufferDescriptor ib{};
    ib.nextInChain      = nullptr;
    ib.label            = sv("mesh_ibo");
    ib.usage            = WGPUBufferUsage_CopyDst | WGPUBufferUsage_Index;
    ib.size             = static_cast<uint64_t>(d.indices.size()) * elemSize;
    ib.mappedAtCreation = false;

    ibo = wgpuDeviceCreateBuffer(dev, &ib);
    if (ib.size && !d.indices.empty()) {
      if (indexFormat == WGPUIndexFormat_Uint16) {
        // down-convert to u16 as needed
        std::vector<uint16_t> tmp; tmp.reserve(d.indices.size());
        for (uint32_t idx : d.indices) tmp.push_back(static_cast<uint16_t>(idx));
        wgpuQueueWriteBuffer(que, ibo, 0, tmp.data(), static_cast<size_t>(ib.size));
      } else {
        wgpuQueueWriteBuffer(que, ibo, 0, d.indices.data(), static_cast<size_t>(ib.size));
      }
    }
  }

  vertexStrideBytes = d.strideFloats * sizeof(float);
  indexCount        = static_cast<uint32_t>(d.indices.size());
  topology          = WGPUPrimitiveTopology_TriangleList;
}

void RenderMesh::Bind(WGPURenderPassEncoder pass) const {
  // Bind vertex + index buffers
  wgpuRenderPassEncoderSetVertexBuffer(
    pass, /*slot*/ 0, vbo, /*offset*/ 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderSetIndexBuffer(
    pass, ibo, indexFormat, /*offset*/ 0, WGPU_WHOLE_SIZE);

  // The pipeline uses this info (topology/indexFormat) via pipeline state;
  // we only bind buffers here. Draw call happens in Engine/Scene code.
}

} // namespace render
