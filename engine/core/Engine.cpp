#include "core/Engine.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "render/WebGPUContext.h"
// ... other includes ...

using render::WebGPUContext;

void Engine::init() {
  // Time::init();  // if you have this
  WebGPUContext::Get().Init();
  WebGPUContext::Get().Configure(1280, 720); // simple initial size
  // create scene, meshes, etc...
}

void Engine::render() {
  auto& ctx = WebGPUContext::Get();
  WGPUTextureView backbuffer = ctx.BeginFrame();
  if (!backbuffer) return; // skip frame if no texture

  // Record commands
  WGPUCommandEncoderDescriptor encDesc{};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(ctx.Device(), &encDesc);

  // Simple render pass that clears to dark grey
  WGPURenderPassColorAttachment color{};
  color.view       = backbuffer;
  color.loadOp     = WGPULoadOp_Clear;
  color.storeOp    = WGPUStoreOp_Store;
  color.clearValue = {0.1f, 0.1f, 0.1f, 1.0f};

  WGPURenderPassDescriptor rp{};
  rp.colorAttachmentCount = 1;
  rp.colorAttachments     = &color;

  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &rp);
  // TODO: bind pipeline + draw your grid once we wire the pipeline
  wgpuRenderPassEncoderEnd(pass);

  WGPUCommandBufferDescriptor cbDesc{};
  WGPUCommandBuffer cb = wgpuCommandEncoderFinish(encoder, &cbDesc);
  wgpuQueueSubmit(ctx.Queue(), 1, &cb);

  // Present
  ctx.EndFrame(backbuffer);

  // Cleanup local handles
  wgpuCommandBufferRelease(cb);
  wgpuCommandEncoderRelease(encoder);
}
