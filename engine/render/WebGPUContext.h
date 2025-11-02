#pragma once
#if defined(FE_WEBGPU)

#include <stdint.h>

// Be tolerant to header location differences on CI
#if __has_include(<webgpu/webgpu.h>)
  #include <webgpu/webgpu.h>
#elif __has_include(<emscripten/webgpu.h>)
  #include <emscripten/webgpu.h>
#else
  #error "WebGPU headers not found (looked for <webgpu/webgpu.h> and <emscripten/webgpu.h>)"
#endif

typedef struct WebGPUContext {
    static WebGPUContext& Get();

    void Init(const char* canvasSelector = "#canvas");
    void ConfigureSurface(uint32_t w, uint32_t h);

    WGPUTextureView BeginFrame();
    void EndFrame(WGPUTextureView view);

    WGPUInstance       instance = nullptr;
    WGPUDevice         device   = nullptr;
    WGPUQueue          queue    = nullptr;
    WGPUSurface        surface  = nullptr;
    WGPUTextureFormat  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

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
