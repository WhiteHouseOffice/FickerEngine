#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "math/MiniMath.h"
#include "render/WebGPUContext.h"
#include <memory>
#include "render/RenderMesh.h"   // declares render::render::g_data
using render::render::g_data;            // make unqualified render::g_data refer to render::render::g_data

// Make WebGPU types visible in this TU (resilient include)
#if defined(FE_WEBGPU)
  #if __has_include(<webgpu/webgpu.h>)
    #include <webgpu/webgpu.h>
  #elif __has_include(<emscripten/webgpu.h>)
    #include <emscripten/webgpu.h>
  #else
    #error "WebGPU headers not found for Engine.cpp"
  #endif
#endif

struct Engine::Impl {
  Time  time;
  Scene scene;
  Game  game;

  void render_frame(uint32_t w, uint32_t h) {
#if defined(FE_WEBGPU)
  auto& ctx = WebGPUContext::Get();
  if (!ctx.device) ctx.Init("#canvas");
  if (!ctx.device) {
    // No WebGPU support: do nothing this frame (console already logs why)
    return;
  }
  ctx.ConfigureSurface(w, h);

    // Compute PV matrix and upload to uniform buffer
    Mat4 P = perspective(60.f * 3.1415926f / 180.f, (float)w / (float)h, 0.01f, 500.f);
    Mat4 V = game.View();
    Mat4 PV = matMul(P, V);
    wgpuQueueWriteBuffer(ctx.queue, ctx.mvpBuffer, 0, PV.m, sizeof(float) * 16);

    // Acquire backbuffer
    WGPUTextureView backView = ctx.BeginFrame();
    if (!backView) return;

    // GPU buffers exported from RenderMesh.cpp
    extern struct GPUData {
      WGPUBuffer vbo; WGPUBuffer ibo; uint32_t indexCount;
      WGPUIndexFormat indexFormat; WGPUPrimitiveTopology topology;
    } render::g_data;

    // Encode and submit render pass
    WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(ctx.device, nullptr);

    WGPURenderPassColorAttachment color = {};
    color.view = backView;
    color.loadOp  = WGPULoadOp_Clear;
    color.storeOp = WGPUStoreOp_Store;
    color.clearValue = {0.05f, 0.05f, 0.06f, 1.0f};

    WGPURenderPassDescriptor rp = {};
    rp.colorAttachmentCount = 1;
    rp.colorAttachments     = &color;

    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(enc, &rp);
    wgpuRenderPassEncoderSetPipeline(pass, ctx.pipeline);
    wgpuRenderPassEncoderSetBindGroup(pass, 0, ctx.bindGroup, 0, nullptr);
    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, render::g_data.vbo, 0, WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderSetIndexBuffer(pass, render::g_data.ibo, render::g_data.indexFormat, 0, WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderDrawIndexed(pass, render::g_data.indexCount, 1, 0, 0, 0);
    wgpuRenderPassEncoderEnd(pass);

    WGPUCommandBuffer cmds = wgpuCommandEncoderFinish(enc, nullptr);
    wgpuQueueSubmit(ctx.queue, 1, &cmds);

    ctx.EndFrame(backView);
#else
    (void)w; (void)h;
#endif
  }
};

Engine& Engine::instance(){ static Engine g; return g; }
Engine::Engine():impl(std::make_unique<Impl>()){}
Engine::~Engine()=default;

void Engine::init(){
  impl->scene.Build();   // builds grid/marker and uploads via RenderMesh
  impl->game.Init();     // sets up free-fly camera, etc.
}

void Engine::stepOnce(){
  const double dt = impl->time.tick();
  const float  clamped = (float)std::min(dt, 0.1);
  impl->game.Update(clamped);
  impl->render_frame(1280, 720);
}

#if defined(FE_WEB)
extern "C" {
  __attribute__((used)) void fe_step_once(){ Engine::instance().stepOnce(); }
}
#endif
