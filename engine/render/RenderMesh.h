#pragma once

#if __has_include(<webgpu/webgpu.h>)
  #include <webgpu/webgpu.h>
#elif __has_include(<emscripten/webgpu.h>)
  #include <emscripten/webgpu.h>
#else
  #error "WebGPU headers not found"
#endif

#include <cstdint>
#include <vector>

struct MeshCPU {
  std::vector<float>    positions;  // xyz per vertex
  std::vector<uint32_t> indices;    // triangles
};

class RenderMesh {
public:
  RenderMesh() = default;
  ~RenderMesh();

  void UploadCPU(WGPUDevice device, const MeshCPU& d);
  void Draw(WGPUDevice device, WGPUQueue queue, WGPUTextureView backbuffer);

  uint32_t indexCount = 0;

private:
  WGPUBuffer vbo = nullptr;
  WGPUBuffer ibo = nullptr;
};
