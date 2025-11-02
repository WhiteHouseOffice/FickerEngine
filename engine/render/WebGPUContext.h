#pragma once
#include <cstdint>

// Make this header compile across different Emscripten versions.
// Prefer emscripten/webgpu.h, then webgpu/webgpu.h, else provide safe typedefs.
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
  // Minimal fake types so IntelliSense / native builds don't error.
  typedef void* WGPUInstance; typedef void* WGPUDevice; typedef void* WGPUQueue;
  typedef void* WGPUSurface;  typedef void* WGPUTexture; typedef void* WGPUTextureView;
  typedef void* WGPUShaderModule; typedef void* WGPURenderPipeline; typedef void* WGPUBuffer;
  typedef void* WGPUBindGroupLayout; typedef void* WGPUBindGroup; typedef void* WGPUPipelineLayout;
  typedef uint32_t WGPUTextureFormat; typedef uint32_t WGPUPresentMode; typedef uint32_t WGPUTextureUsageFlags;
  typedef uint32_t WGPUBufferUsageFlags; typedef uint32_t WGPUShaderStageFlags; typedef uint32_t WGPUBufferBindingType;
  typedef uint32_t WGPUColorWriteMaskFlags; typedef uint32_t WGPUPrimitiveTopology; typedef uint32_t WGPUVertexFormat;
  typedef uint32_t WGPUFrontFace; typedef uint32_t WGPUCullMode; typedef uint32_t WGPUIndexFormat;

  #ifndef WGPUTextureFormat_BGRA8Unorm
  #define WGPUTextureFormat_BGRA8Unorm 87u
  #endif
  #ifndef WGPUPresentMode_Fifo
  #define WGPUPresentMode_Fifo 2u
  #endif
  #ifndef WGPUTextureUsage_RenderAttachment
  #define WGPUTextureUsage_RenderAttachment 0x10u
  #endif
  #ifndef WGPUBufferUsage_Uniform
  #define WGPUBufferUsage_Uniform 0x10u
  #endif
  #ifndef WGPUBufferUsage_CopyDst
  #define WGPUBufferUsage_CopyDst 0x20u
  #endif
  #ifndef WGPUBufferUsage_Vertex
  #define WGPUBufferUsage_Vertex 0x1u
  #endif
  #ifndef WGPUBufferUsage_Index
  #define WGPUBufferUsage_Index 0x2u
  #endif
  #ifndef WGPUShaderStage_Vertex
  #define WGPUShaderStage_Vertex 0x1u
  #endif
  #ifndef WGPUBufferBindingType_Uniform
  #define WGPUBufferBindingType_Uniform 0u
  #endif
  #ifndef WGPUColorWriteMask_All
  #define WGPUColorWriteMask_All 0xFu
  #endif
  #ifndef WGPUPrimitiveTopology_LineList
  #define WGPUPrimitiveTopology_LineList 1u
  #endif
  #ifndef WGPUVertexFormat_Float32x3
  #define WGPUVertexFormat_Float32x3 3u
  #endif
  #ifndef WGPUFrontFace_CCW
  #define WGPUFrontFace_CCW 0u
  #endif
  #ifndef WGPUCullMode_None
  #define WGPUCullMode_None 0u
  #endif
  #ifndef WGPUIndexFormat_Uint32
  #define WGPUIndexFormat_Uint32 2u
  #endif
#endif

class WebGPUContext {
public:
  static WebGPUContext& Get();

  void Init(const char* canvasSelector = "#canvas");
  void ConfigureSurface(uint32_t w, uint32_t h);
  WGPUTextureView BeginFrame();
  void EndFrame(WGPUTextureView view);

  // Exposed handles
  WGPUInstance       instance = nullptr;
  WGPUDevice         device   = nullptr;
  WGPUQueue          queue    = nullptr;
  WGPUSurface        surface  = nullptr;
  WGPUTextureFormat  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  // Minimal pipeline resources
  WGPUBuffer             mvpBuffer      = nullptr;
  WGPUBindGroupLayout    bindGroupLayout= nullptr;
  WGPUBindGroup          bindGroup      = nullptr;
  WGPUPipelineLayout     pipelineLayout = nullptr;
  WGPUShaderModule       shader         = nullptr;
  WGPURenderPipeline     pipeline       = nullptr;

  uint32_t width = 0, height = 0;
  bool     initialized = false;

private:
  void createDevice_();
  void createSurface_(const char* canvasSelector);
  void createMVPResources_();
  void createPipeline_();
};
