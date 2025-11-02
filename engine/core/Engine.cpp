#include "core/Engine.h"
#include "core/Time.h"
#include "render/WebGPUContext.h"

#if defined(FE_WEBGPU)
  // we will rely on WebGPUContext methods
#endif

Engine::Engine() = default;
Engine::~Engine() = default;

void Engine::init() {
#if defined(FE_WEBGPU)
    // Initialize WebGPU (canvas selector can be adjusted if your HTML differs)
    WebGPUContext::Get().Init("#canvas");
#endif
    Time::init();
}

void Engine::update() {
    Time::update();
    // float dt = Time::deltaTime();  // keep if you need it later
    // (Game/Scene updates intentionally omitted for now to keep build green)
}

void Engine::render() {
#if defined(FE_WEBGPU)
    auto& ctx = WebGPUContext::Get();
    if (!ctx.device || !ctx.queue || !ctx.surface) return;

    // Begin frame: acquire the swapchain texture view
    WGPUTextureView view = ctx.BeginFrame();
    if (!view) return;

    // Command encoder
    WGPUCommandEncoderDescriptor encDesc{};
    WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(ctx.device, &encDesc);

    // Simple clear pass (neutral gray background)
    WGPURenderPassColorAttachment color{};
    color.view = view;
    color.loadOp = WGPULoadOp_Clear;
    color.storeOp = WGPUStoreOp_Store;
    color.clearValue = {0.3f, 0.3f, 0.3f, 1.0f};

    WGPURenderPassDescriptor rpDesc{};
    rpDesc.colorAttachmentCount = 1;
    rpDesc.colorAttachments = &color;

    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &rpDesc);
    wgpuRenderPassEncoderEnd(pass);

    // Submit
    WGPUCommandBufferDescriptor cbDesc{};
    WGPUCommandBuffer cb = wgpuCommandEncoderFinish(encoder, &cbDesc);
    wgpuQueueSubmit(ctx.queue, 1, &cb);

    // Present + cleanup
    wgpuCommandBufferRelease(cb);
    wgpuCommandEncoderRelease(encoder);

    ctx.EndFrame(view);
#endif
}

void Engine::shutdown() {
#if defined(FE_WEBGPU)
    WebGPUContext::Get().Shutdown();
#endif
}
