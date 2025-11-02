#pragma once
#include <cstdint>
#include <vector>

#if defined(__EMSCRIPTEN__)
  #if __has_include(<emscripten/webgpu.h>)
    #include <emscripten/webgpu.h>
  #elif __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #else
    #define FE_WGPU_FAKE 1
  #endif
#else
  #if __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #else
    #define FE_WGPU_FAKE 1
  #endif
#endif

#ifdef FE_WGPU_FAKE
  typedef void* WGPUBuffer;
  typedef uint32_t WGPUIndexFormat;
  typedef uint32_t WGPUPrimitiveTopology;
  #ifndef WGPUIndexFormat_Uint32
  #define WGPUIndexFormat_Uint32 2u
  #endif
  #ifndef WGPUPrimitiveTopology_LineList
  #define WGPUPrimitiveTopology_LineList 1u
  #endif
#endif

namespace render {

struct MeshUpload {
    std::vector<float>    positions;  // xyz packed
    std::vector<uint32_t> indices;    // line list
};

void UploadGrid(const MeshUpload& m);

struct GPUData {
    WGPUBuffer vbo = nullptr;
    WGPUBuffer ibo = nullptr;
    uint32_t   indexCount = 0;
    WGPUIndexFormat indexFormat;        // set in .cpp
    WGPUPrimitiveTopology topology;     // set in .cpp
};
extern GPUData g_data;

} // namespace render
