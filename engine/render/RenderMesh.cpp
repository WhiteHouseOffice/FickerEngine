#include "RenderMesh.h"
#include "WebGPUContext.h"
#include <cstring>   // for std::memcpy (if ever needed)
#include <cstdio>

namespace render {

void RenderMesh::Release() {
  if (vbo) { wgpuBufferRelease(vbo); vbo = nullptr; }
  if (ibo) { wgpuBufferRelease(ibo); ibo = nullptr; }
  indexCount   = 0;
  indexFormat  = WGPUIndexFormat_Undefined;
  topology     = WGPUPrimitiveTopology_TriangleList;
}

void RenderMesh::UploadCPU(const MeshData& d) {
  auto& ctx = WebGPUContext::Get();
  auto dev  = ctx.Device();
  auto q    = ctx.Queue();

  if (!dev || !q) {
    std::printf("[RenderMesh] UploadCPU skipped: device/queue not ready.\n");
    return;
  }

  // reset previous GPU objects
  Release();

  // --- Vertex buffer ---
  {
    WGPUBufferDescriptor vb{};
    vb.nextInChain      = nullptr;
    // Do not assign vb.label directly (Dawn uses WGPUStringView) — omit for simplicity.
    vb.usage            = WGPUBufferUsage_CopyDst | WGPUBufferUsage_Vertex;
    vb.size             = static_cast<uint64_t>(d.positions.size() * sizeof(float));
    vb.mappedAtCreation = false;

    vbo = wgpuDeviceCreateBuffer(dev, &vb);
    if (vbo && vb.size > 0) {
      wgpuQueueWriteBuffer(q, vbo, 0, d.positions.data(), vb.size);
    }
  }

  // --- Index buffer ---
  {
    WGPUBufferDescriptor ib{};
    ib.nextInChain      = nullptr;
    ib.usage            = WGPUBufferUsage_CopyDst | WGPUBufferUsage_Index;
    ib.size             = static_cast<uint64_t>(d.indices.size() * sizeof(uint32_t));
    ib.mappedAtCreation = false;

    ibo = wgpuDeviceCreateBuffer(dev, &ib);
    if (ibo && ib.size > 0) {
      wgpuQueueWriteBuffer(q, ibo, 0, d.indices.data(), ib.size);
    }

    indexCount  = static_cast<uint32_t>(d.indices.size());
    indexFormat = WGPUIndexFormat_Uint32;
  }

  // Primitive topology (keep triangle list as default)
  topology = WGPUPrimitiveTopology_TriangleList;
}

// Bind vertex/index buffers on an already-open render pass.
// The pipeline (with correct vertex layout) must be set by the caller.
void RenderMesh::Bind(WGPURenderPassEncoder pass) const {
  if (!pass || !vbo || !ibo || indexCount == 0) return;

  // one interleaved attribute stream at slot 0 (positions-only layout assumed in pipeline)
  wgpuRenderPassEncoderSetVertexBuffer(pass, /*slot*/ 0, vbo, /*offset*/ 0, /*size*/ WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderSetIndexBuffer (pass, ibo, indexFormat, /*offset*/ 0, /*size*/ WGPU_WHOLE_SIZE);
}

// Issue a draw call (expects Bind() to have been called or caller sets buffers/pipeline).
void RenderMesh::Draw(WGPURenderPassEncoder pass) const {
  if (!pass || indexCount == 0) return;
  wgpuRenderPassEncoderDrawIndexed(pass, indexCount, /*instanceCount*/ 1, /*firstIndex*/ 0, /*baseVertex*/ 0, /*firstInstance*/ 0);
}

} // namespace render
