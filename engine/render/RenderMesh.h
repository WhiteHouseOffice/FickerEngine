#pragma once
#include <cstdint>
#include <vector>

#if defined(FE_WEBGPU)
  #if __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #elif __has_include(<emscripten/webgpu.h>)
    #include <emscripten/webgpu.h>
  #endif
#endif

namespace render {

struct MeshUpload {
    std::vector<float>    positions;  // xyz packed
    std::vector<uint32_t> indices;    // line list
};

void UploadGrid(const MeshUpload& m);

struct GPUData {
#if defined(FE_WEBGPU)
    WGPUBuffer vbo = nullptr;
    WGPUBuffer ibo = nullptr;
    uint32_t   indexCount = 0;
    WGPUIndexFormat indexFormat;           // set in .cpp
    WGPUPrimitiveTopology topology;        // set in .cpp
#else
    void* _ = nullptr;
#endif
};
extern GPUData g_data;

} // namespace render
