#pragma once
#if defined(FE_WEBGPU)

#include <webgpu/webgpu.h>
#include <stdint.h>

typedef struct WebGPUContext {
    static WebGPUContext& Get();

    void Init(const char* canvasSelector = "#canvas");
    void ConfigureSurface(uint32_t w, uint32_t h);

    WGPUTextureView BeginFrame();   // returns null on failure
    void EndFrame(WGPUTextureView view);

    // Core
    WGPUInstance       instance = nullptr;
    WGPUDevice         device   = nullptr;
    WGPUQueue          queue    = nullptr;
    WGPUSurface        surface  = nullptr;
    WGPUTextureFormat  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

    // MVP uniform + pipeline
    WGPUBuffer             mvpBuffer      = nullptr;
    WGPUBindGroupLayout    bindGroupLayout= nullptr;
    WGPUBindGroup          bindGroup      = nullptr;
    WGPUPipelineLayout     pipelineLayout = nullptr;
    WGPUShaderModule       shader         = nullptr;
    WGPURenderPipeline     pipeline       = nullptr;

    uint32_t width = 1280, height = 720;

private:
    bool initialized = false;
    void createDevice_();
    void createSurface_(const char* canvasSelector);
    void configureSurface_();
    void createMVPResources_();
    void createPipeline_();
} WebGPUContext;

#endif // FE_WEBGPU
