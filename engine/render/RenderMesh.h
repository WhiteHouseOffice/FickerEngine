#pragma once
#include <cstdint>
#include <vector>

// Try real WebGPU headers first; fall back to safe dummies for editors/CI.
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
  // Minimal fake types/flags so the file compiles without real headers.
  typedef void* WGPUDevice;
  typedef void* WGPUQueue;
  typedef void* WGPUBuffer;
  typedef uint32_t WGPUIndexFormat;
  typedef uint32_t WGPUPrimitiveTopology;
  typedef uint32_t WGPUBufferUsageFlags;

  #ifndef WGPUIndexFormat_Uint32
  #define WGPUIndexFormat_Uint32 2u
  #endif
  #ifndef WGPUPrimitiveTopology_LineList
  #define WGPUPrimitiveTopology_LineList 1u
  #endif
  #ifndef WGPUBufferUsage_Vertex
  #define WGPUBufferUsage_Vertex 0x1u
  #endif
  #ifndef WGPUBufferUsage_Index
  #define WGPUBufferUsage_Index 0x2u
  #endif
  #ifndef WGPUBufferUsage_CopyDst
  #define WGPUBufferUsage_CopyDst 0x20u
  #endif

  typedef struct WGPUBufferDescriptor {
    const void*  nextInChain;
    const char*  label;
    uint64_t     size;
    WGPUBufferUsageFlags usage;
    bool         mappedAtCreation;
  } WGPUBufferDescriptor;

  // Prototypes so we can call them even without including the real header.
  extern "C" {
    WGPUBuffer wgpuDeviceCreateBuffer(WGPUDevice device, const WGPUBufferDescriptor* desc);
    void wgpuQueueWriteBuffer(WGPUQueue queue, WGPUBuffer buffer, uint64_t bufferOffset,
                              const void* data, size_t size);
  }
#endif // FE_WGPU_FAKE

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
