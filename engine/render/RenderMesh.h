#pragma once
#include <cstdint>
#include <vector>

#if defined(FE_WEBGPU)
  #if defined(__EMSCRIPTEN__)
    #include <emscripten/webgpu.h>
  #else
    #include <webgpu/webgpu.h>
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
    WGPUIndexFormat indexFormat;        // set in .cpp
    WGPUPrimitiveTopology topology;     // set in .cpp
#else
    void* _ = nullptr;
#endif
};
extern GPUData g_data;

} // namespace render
