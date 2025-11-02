#pragma once
#include <cstdint>

#if defined(FE_WEBGPU)
  #if defined(__EMSCRIPTEN__)
    #include <emscripten/webgpu.h>
  #else
    // If you're ever compiling a native WebGPU backend, include your wgpu.h here:
    // #include <webgpu/webgpu.h>
    // For editor friendliness, provide typedef fallbacks (no-ops outside web):
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
    typedef unsigned int WGPUPresentMode;
    typedef unsigned int WGPUColorWriteMask;
    typedef unsigned int WGPUPrimitiveTopology;
    #ifndef WGPUTextureFormat_BGRA8Unorm
    #define WGPUTextureFormat_BGRA8Unorm 87u
    #endif
    #ifndef WGPUPresentMode_Fifo
    #define WGPUPresentMode_Fifo 2u
    #endif
    #ifndef WGPUColorWriteMask_All
    #define WGPUColorWriteMask_All 0xF
    #endif
    #ifndef WGPUPrimitiveTopology_LineList
    #define WGPUPrimitiveTopology_LineList 1u
    #endif
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
    WGPUShaderModule       shader         = 0;
    WGPURenderPipeline     pipeline       = 0;

    uint32_t width = 0, height = 0;
    bool     initialized = false;

private:
    void createDevice_();
    void createSurface_(const char* canvasSelector);
    void createMVPResources_();
    void createPipeline_();
};
