#pragma once
#if defined(FE_WEBGPU)

#include <emscripten/html5_webgpu.h>
#include <webgpu/webgpu.h>
#include <stdint.h>

typedef struct WebGPUContext {
    // singleton accessor
    static WebGPUContext& Get();

    // init once; sets up instance, device, surface, MVP uniform, pipeline
    void Init(const char* canvasSelector = "#canvas");

    // resize/configure surface
    void ConfigureSurface(uint32_t w, uint32_t h);

    // per-frame
    WGPUTextureView BeginFrame();                 // returns null on failure
    void EndFrame(WGPUTextureView view);

    // public GPU objects we use elsewhere
    WGPUInstance       instance = nullptr;
    WGPUDevice         device   = nullptr;
    WGPUQueue          queue    = nullptr;
    WGPUSurface        surface  = nullptr;
    WGPUTextureFormat  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

    // basic MVP resources
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
