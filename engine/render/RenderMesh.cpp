#include "render/RenderMesh.h"
#include "render/WebGPUContext.h"

namespace render {

GPUData g_data = { nullptr, nullptr, 0u, WGPUIndexFormat_Uint32, WGPUPrimitiveTopology_LineList };

void UploadGrid(const MeshUpload& m) {
#if defined(FE_WEBGPU)
    auto& ctx = WebGPUContext::Get();
    if (!ctx.device) return;

    // Create / update VBO
    if (g_data.vbo) wgpuBufferRelease(g_data.vbo);
    {
        WGPUBufferDescriptor bd = {};
        bd.size  = m.positions.size() * sizeof(float);
        bd.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
        g_data.vbo = wgpuDeviceCreateBuffer(ctx.device, &bd);
        wgpuQueueWriteBuffer(ctx.queue, g_data.vbo, 0, m.positions.data(), bd.size);
    }

    // Create / update IBO
    if (g_data.ibo) wgpuBufferRelease(g_data.ibo);
    {
        WGPUBufferDescriptor bd = {};
        bd.size  = m.indices.size() * sizeof(uint32_t);
        bd.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
        g_data.ibo = wgpuDeviceCreateBuffer(ctx.device, &bd);
        wgpuQueueWriteBuffer(ctx.queue, g_data.ibo, 0, m.indices.data(), bd.size);
    }

    g_data.indexCount = static_cast<uint32_t>(m.indices.size());
    g_data.indexFormat = WGPUIndexFormat_Uint32;
    g_data.topology = WGPUPrimitiveTopology_LineList;
#endif
}

} // namespace render
