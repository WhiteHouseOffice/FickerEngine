#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "game/GameObject.h"
#include "render/RenderMesh.h"
#include "render/WebGPUContext.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include <memory>
#include <cstdio>

// --- WebGPU headers (types only) ---
#if __has_include(<emscripten/webgpu.h>)
  #include <emscripten/webgpu.h>
#elif __has_include(<webgpu/webgpu.h>)
  #include <webgpu/webgpu.h>
#endif

// --- Context wrapper (your file) ---
#if __has_include("render/WebGPUContext.h")
  #include "render/WebGPUContext.h"
#endif

#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h> // emscripten_get_now
#endif

namespace {
  // Minimal per-frame timing (independent from any Time module)
  double g_last = 0.0;
  float  g_dt   = 0.0f;

  inline double now_seconds() {
  #ifdef __EMSCRIPTEN__
    return emscripten_get_now() * 0.001; // ms -> s
  #else
    return 0.0; // not used in web-only flow
  #endif
  }

  inline void tick_dt() {
    const double t = now_seconds();
    if (g_last == 0.0) {
      g_last = t;
      g_dt   = 0.0f;
      return;
    }
    double d = t - g_last;
    g_last = t;
    if (d < 0.0) d = 0.0;
    if (d > 0.2) d = 0.2; // cap to avoid huge dt after tab sleep
    g_dt = static_cast<float>(d);
  }
}

Engine::Engine() = default;
Engine::~Engine() = default;

void Engine::init() {
#if defined(FE_WEBGPU) && __has_include("render/WebGPUContext.h")
  // Initialize the WebGPU context; adjust selector to your HTML if needed.
  WebGPUContext::Get().Init("#canvas");
#endif
  // start timer
  g_last = now_seconds();
  g_dt   = 0.0f;
}

void Engine::update() {
  tick_dt();
  // You can consume g_dt if you need it for movement later.
  (void)g_dt;
}

void Engine::render() {
#if defined(FE_WEBGPU) && __has_include("render/WebGPUContext.h")
  auto& ctx = WebGPUContext::Get();
  if (!ctx.device || !ctx.queue || !ctx.surface) return;

  // Acquire current frame (swapchain view)
  WGPUTextureView view = ctx.BeginFrame();
  if (!view) return;

  // Command encoder
  WGPUCommandEncoderDescriptor encDesc{};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(ctx.device, &encDesc);

  // Clear to neutral gray
  WGPURenderPassColorAttachment color{};
  color.view = view;
  color.loadOp = WGPULoadOp_Clear;
  color.storeOp = WGPUStoreOp_Store;
  color.clearValue = {0.30f, 0.30f, 0.30f, 1.0f};

  WGPURenderPassDescriptor rpDesc{};
  rpDesc.colorAttachmentCount = 1;
  rpDesc.colorAttachments = &color;

  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &rpDesc);
  wgpuRenderPassEncoderEnd(pass);

  // Submit
  WGPUCommandBufferDescriptor cbDesc{};
  WGPUCommandBuffer cb = wgpuCommandEncoderFinish(encoder, &cbDesc);
  wgpuQueueSubmit(ctx.queue, 1, &cb);

  // Cleanup + present
  wgpuCommandBufferRelease(cb);
  wgpuCommandEncoderRelease(encoder);
  ctx.EndFrame(view);
#endif
}

// Intentionally empty: some headers don't expose a Shutdown() method yet.
void Engine::shutdown() {
  // no-op for now
}
