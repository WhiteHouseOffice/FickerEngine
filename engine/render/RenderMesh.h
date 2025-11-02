#pragma once
#include <cstdint>
#include <vector>

#if defined(FE_WEBGPU)
  #if __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #elif __has_include(<emscripten/webgpu.h>)
    #include <emscripten/webgpu.h>
  #else
    // Forward-declare minimal types so native builds still compile
    typedef void* WGPUBuffer;
    typedef unsigned int WGPUIndexFormat;
    typedef unsigned int WGPUPrimitiveTopology;
  #endif
#endif

namespace render {

// CPU-side payload
struct MeshUpload {
    std::vector<float>    positions;  // xyz packed
    std::vector<uint32_t> indices;    // line list
};

// Upload/replace the single grid mesh buffers
void UploadGrid(const MeshUpload& m);

// Renderer-visible GPU handles (defined in RenderMesh.cpp)
struct GPUData {
#if defined(FE_WEBGPU)
    WGPUBuffer vbo = nullptr;
    WGPUBuffer ibo = nullptr;
    uint32_t   indexCount = 0;
    WGPUIndexFormat indexFormat = 0;
    WGPUPrimitiveTopology topology = 0;
#else
    // Native stub
    void* _ = nullptr;
#endif
};
extern GPUData g_data;

} // namespace render