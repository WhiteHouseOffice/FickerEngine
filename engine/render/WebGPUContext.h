#pragma once
#include <cstdint>

#if defined(FE_WEBGPU)
  #if __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #elif __has_include(<emscripten/webgpu.h>)
    #include <emscripten/webgpu.h>
  #else
    // Minimal fallbacks so native compiles; not used at runtime
    typedef void* WGPUInstance;
    typedef void* WGPUDevice;
    typedef void* WGPUQueue;
    typedef void* WGPUSurface;
    typedef void* WGPUTextureView;
    typedef unsigned int WGPUTextureFormat;
    typedef unsigned int WGPUShaderModule;
    typedef unsigned int WGPURenderPipeline;
    typedef unsigned int WGPUBuffer;
    typedef unsigned int WGPUBindGroupLayout;
    typedef unsigned int WGPUBindGroup;
    typedef unsigned int WGPUPipelineLayout;
    #define WGPUTextureFormat_BGRA8Unorm 87u
  #endif
#endif

class WebGPUContext {
public:
    static WebGPUContext& Get();

    void Init(const char* canvasSelector = "#canvas");
    void ConfigureSurface(uint32_t w, uint32_t h);
    WGPUTextureView BeginFrame();
    void EndFrame(WGPUTextureView view);

    // Exposed so Engine can write uniforms / submit
    WGPUInstance       instance = nullptr;
    WGPUDevice         device   = nullptr;
    WGPUQueue          queue    = nullptr;
    WGPUSurface        surface  = nullptr;
    WGPUTextureFormat  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

    // Simple uniform + pipeline for the grid
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

