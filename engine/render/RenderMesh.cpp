#include "render/RenderMesh.h"
#include "render/WebGPUContext.h"

#include <cstring>

static inline WGPUStringView sv(const char* s) {
  WGPUStringView r;
  r.data = s;
  r.length = (uint64_t)std::strlen(s);
  return r;
}

RenderMesh::~RenderMesh() {
  if (vbo) wgpuBufferRelease(vbo);
  if (ibo) wgpuBufferRelease(ibo);
}

void RenderMesh::UploadCPU(WGPUDevice device, const MeshCPU& d) {
  // Vertex buffer
  WGPUBufferDescriptor vb{};
  vb.size = d.positions.size() * sizeof(float);
  vb.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_Vertex;
  vb.mappedAtCreation = false;
  vb.label = sv("grid_vbo");
  vbo = wgpuDeviceCreateBuffer(device, &vb);

  // Index buffer
  WGPUBufferDescriptor ib{};
  ib.size = d.indices.size() * sizeof(uint32_t);
  ib.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_Index;
  ib.mappedAtCreation = false;
  ib.label = sv("grid_ibo");
  ibo = wgpuDeviceCreateBuffer(device, &ib);

  // Upload via queue
  WGPUQueue q = wgpuDeviceGetQueue(device);
  if (!d.positions.empty())
    wgpuQueueWriteBuffer(q, vbo, 0, d.positions.data(), d.positions.size() * sizeof(float));
  if (!d.indices.empty())
    wgpuQueueWriteBuffer(q, ibo, 0, d.indices.data(), d.indices.size() * sizeof(uint32_t));

  indexCount = static_cast<uint32_t>(d.indices.size());
}

// NOTE: This is a stub; proper pipeline creation (shaders, layouts) comes next.
// For now we just ensure the buffer objects exist and the swapchain is alive.
void RenderMesh::Draw(WGPUDevice /*device*/, WGPUQueue /*queue*/, WGPUTextureView /*backbuffer*/) {
  // Will implement once minimal pipeline is wired.
  (void)indexCount;
}
