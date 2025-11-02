#pragma once
#include <cstdint>

#if defined(FE_WEBGPU)
  #if defined(__EMSCRIPTEN__)
    #include <emscripten/webgpu.h>
  #else
    #include <webgpu/webgpu.h>
  #endif
#endif

class WebGPUContext {
public:
    static WebGPUContext& Get();

    void Init(const char* canvasSelector = "#canvas");
    void ConfigureSurface(uint32_t w, uint32_t h);
    WGPUTextureView BeginFrame();
    void EndFrame(WGPUTextureView view);

    // Exposed to engine
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
