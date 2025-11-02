#include "RenderMesh.h"

#if defined(FE_WEBGPU)
#include "WebGPUContext.h"
#include <stdint.h>

struct GPUData {
    WGPUBuffer vbo;
    WGPUBuffer ibo;
    uint32_t   indexCount;
    WGPUIndexFormat indexFormat;
    WGPUPrimitiveTopology topology;
};
GPUData g_data = {};

bool RenderMesh::Upload(const MeshData& d) {
    auto& ctx = WebGPUContext::Get();
    ctx.Init("#canvas");

    // vertex buffer
    {
        WGPUBufferDescriptor bd = {};
        bd.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
        bd.size  = d.positions.size() * sizeof(float);
        g_data.vbo = wgpuDeviceCreateBuffer(ctx.device, &bd);
        wgpuQueueWriteBuffer(ctx.queue, g_data.vbo, 0, d.positions.data(), bd.size);
    }

    // index buffer
    {
        WGPUBufferDescriptor bd = {};
        bd.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
        bd.size  = d.indices.size() * sizeof(uint32_t);
        g_data.ibo = wgpuDeviceCreateBuffer(ctx.device, &bd);
        wgpuQueueWriteBuffer(ctx.queue, g_data.ibo, 0, d.indices.data(), bd.size);
        g_data.indexCount  = (uint32_t)d.indices.size();
        g_data.indexFormat = WGPUIndexFormat_Uint32;
    }

    g_data.topology = (d.topology == MeshData::Topology::Lines)
        ? WGPUPrimitiveTopology_LineList
        : WGPUPrimitiveTopology_TriangleList;

    return true;
}

void RenderMesh::Draw(float, float, float) const {
    // no-op; Engine issues the draw using the shared pipeline
}

#endif // FE_WEBGPU
