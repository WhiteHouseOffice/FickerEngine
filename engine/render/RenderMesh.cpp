#include "render/RenderMesh.h"
#include "render/WebGPUContext.h"

namespace render {

GPUData g_data; // ← this provides the definition the linker needs

void UploadGrid(const MeshUpload& m) {
    auto& ctx = WebGPUContext::Get();
    if (!ctx.device || m.positions.empty() || m.indices.empty()) {
        g_data = {}; // keep safe defaults
        return;
    }

    // Vertex buffer
    WGPUBufferDescriptor vb{};
    vb.label = "grid_vbo";
    vb.size  = static_cast<uint64_t>(sizeof(float) * m.positions.size());
    vb.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
    vb.mappedAtCreation = false;
    g_data.vbo = wgpuDeviceCreateBuffer(ctx.device, &vb);
    wgpuQueueWriteBuffer(ctx.queue, g_data.vbo, 0, m.positions.data(), vb.size);

    // Index buffer
    WGPUBufferDescriptor ib{};
    ib.label = "grid_ibo";
    ib.size  = static_cast<uint64_t>(sizeof(uint32_t) * m.indices.size());
    ib.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
    ib.mappedAtCreation = false;
    g_data.ibo = wgpuDeviceCreateBuffer(ctx.device, &ib);
    wgpuQueueWriteBuffer(ctx.queue, g_data.ibo, 0, m.indices.data(), ib.size);

    g_data.indexCount  = static_cast<uint32_t>(m.indices.size());
    g_data.indexFormat = WGPUIndexFormat_Uint32;
    g_data.topology    = WGPUPrimitiveTopology_LineList;
}

} // namespace render
