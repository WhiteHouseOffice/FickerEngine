#pragma once
#if defined(FE_WEBGPU)
  #if __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #elif __has_include(<emscripten/webgpu.h>)
    #include <emscripten/webgpu.h>
  #endif
#endif
#include <cstdint>
#include <vector>

namespace render {

struct MeshUpload {
    std::vector<float>    positions;  // xyz packed
    std::vector<uint32_t> indices;    // line list
};

void UploadGrid(const MeshUpload& m);

// Expose a tiny struct the renderer uses (declared in .cpp)
struct GPUData {
    WGPUBuffer vbo;
    WGPUBuffer ibo;
    uint32_t   indexCount;
    WGPUIndexFormat indexFormat;
    WGPUPrimitiveTopology topology;
};
extern GPUData g_data;

} // namespace render
