#include "RenderMesh.h"

#if defined(FE_WEBGPU)

#include "WebGPUContext.h"
#include <webgpu/webgpu_cpp.h>
#include <cstring>

struct GPUData {
    wgpu::Buffer vbo;
    wgpu::Buffer ibo;
    uint32_t indexCount = 0;
    wgpu::IndexFormat indexFormat = wgpu::IndexFormat::Uint32;
    wgpu::PrimitiveTopology topology = wgpu::PrimitiveTopology::LineList;
};
static GPUData g_data;

bool RenderMesh::Upload(const MeshData& d) {
    auto& ctx = WebGPUContext::Get();
    ctx.Init("#canvas");

    // Vertex buffer
    {
        wgpu::BufferDescriptor bd{};
        bd.size  = d.positions.size() * sizeof(float);
        bd.usage = wgpu::BufferUsage::Vertex | wgpu::BufferUsage::CopyDst;
        g_data.vbo = ctx.device.CreateBuffer(&bd);
        ctx.queue.WriteBuffer(g_data.vbo, 0, d.positions.data(), bd.size);
    }

    // Index buffer
    {
        wgpu::BufferDescriptor bd{};
        bd.size  = d.indices.size() * sizeof(uint32_t);
        bd.usage = wgpu::BufferUsage::Index | wgpu::BufferUsage::CopyDst;
        g_data.ibo = ctx.device.CreateBuffer(&bd);
        ctx.queue.WriteBuffer(g_data.ibo, 0, d.indices.data(), bd.size);
        g_data.indexCount = (uint32_t)d.indices.size();
        g_data.indexFormat = wgpu::IndexFormat::Uint32;
    }

    // Topology
    g_data.topology = (d.topology == MeshData::Topology::Lines)
        ? wgpu::PrimitiveTopology::LineList
        : wgpu::PrimitiveTopology::TriangleList;

    return true;
}

void RenderMesh::Draw(float /*r*/, float /*g*/, float /*b*/) const {
    // Drawing is batched in Engine per-frame using our shared pipeline.
    // Here we don’t issue commands; Engine will bind our buffers.
}

#else
// If not WebGPU, keep the old (WebGL / native) paths here if you still need them.
// For now, we’re moving web to WebGPU, so this branch can be left empty or assert.
bool RenderMesh::Upload(const MeshData&) { return true; }
void RenderMesh::Draw(float,float,float) const {}
#endif
