#pragma once
#if defined(FE_WEBGPU)

#include <webgpu/webgpu_cpp.h>
#include <vector>

struct WebGPUContext {
    // Singleton
    static WebGPUContext& Get();

    // Ensure device/surface/pipeline exist (idempotent)
    void Init(const char* canvasSelector = "#canvas");

    // Resize swapchain/surface config (call if canvas size changes)
    void ConfigureSurface(uint32_t width, uint32_t height);

    // Begin a frame; returns a valid view or a null view if lost
    wgpu::TextureView BeginFrame();
    void EndFrame(const wgpu::TextureView& view);

    // Uniform buffer for a single 4x4 MVP matrix
    wgpu::Buffer mvpBuffer;

    // Pipeline for colored lines/triangles (unlit)
    wgpu::RenderPipeline pipeline;

    // Bind group layout & bind group for MVP
    wgpu::BindGroupLayout bindGroupLayout;
    wgpu::BindGroup       bindGroup;

    // Format
    wgpu::TextureFormat surfaceFormat = wgpu::TextureFormat::BGRA8Unorm;

    // Device/Queue
    wgpu::Device device;
    wgpu::Queue  queue;

    // Surface
    wgpu::Instance instance;
    wgpu::Surface  surface;

    // Cached size
    uint32_t width = 1280, height = 720;

private:
    bool initialized = false;
    wgpu::ShaderModule shader;
    wgpu::PipelineLayout pipelineLayout;

    void createDevice_();
    void createSurface_(const char* canvasSelector);
    void configureSurface_();
    void createMVPResources_();
    void createPipeline_();
};

#endif // FE_WEBGPU
