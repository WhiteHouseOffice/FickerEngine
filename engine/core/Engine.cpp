#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "math/MiniMath.h"
#include "render/RenderMesh.h"

#include <memory>

#if defined(FE_WEBGPU)
  #include "render/WebGPUContext.h"
  #include <webgpu/webgpu_cpp.h>
#endif

struct Engine::Impl {
  Time  time;
  Scene scene;
  Game  game;

  void render_frame(uint32_t w,uint32_t h){
#if defined(FE_WEBGPU)
    auto& ctx = WebGPUContext::Get();
    if (!ctx.device) ctx.Init("#canvas");
    ctx.ConfigureSurface(w,h);

    // Acquire backbuffer
    auto backView = ctx.BeginFrame();
    if (!backView) return;

    // P * V
    Mat4 P = perspective(60.f*3.1415926f/180.f, (float)w/(float)h, 0.01f, 500.f);
    Mat4 V = game.View();
    Mat4 PV = matMul(P, V);
    ctx.queue.WriteBuffer(ctx.mvpBuffer, 0, PV.m, sizeof(float)*16);

    // Encoder / pass
    wgpu::CommandEncoder enc = ctx.device.CreateCommandEncoder();
    wgpu::RenderPassColorAttachment color{};
    color.view = backView;
    color.clearValue = {0.05f,0.05f,0.06f,1.0f};
    color.loadOp  = wgpu::LoadOp::Clear;
    color.storeOp = wgpu::StoreOp::Store;

    wgpu::RenderPassDescriptor rp{};
    rp.colorAttachmentCount = 1;
    rp.colorAttachments = &color;

    wgpu::RenderPassEncoder pass = enc.BeginRenderPass(&rp);
    pass.SetPipeline(ctx.pipeline);
    pass.SetBindGroup(0, ctx.bindGroup);

    // Draw all objects as lines (grid + marker)
    // We rely on the last uploaded mesh buffers stored in static GPUData inside RenderMesh.cpp.
    extern struct GPUData { wgpu::Buffer vbo; wgpu::Buffer ibo; uint32_t indexCount; wgpu::IndexFormat indexFormat; wgpu::PrimitiveTopology topology; } g_data;
    pass.SetVertexBuffer(0, g_data.vbo, 0, WGPU_WHOLE_SIZE);
    pass.SetIndexBuffer(g_data.ibo, g_data.indexFormat, 0, WGPU_WHOLE_SIZE);
    pass.DrawIndexed(g_data.indexCount);

    pass.End();

    auto cmds = enc.Finish();
    ctx.queue.Submit(1, &cmds);

    ctx.EndFrame(backView);
#else
    // Native / other paths could go here later
    (void)w; (void)h;
#endif
  }
};

Engine& Engine::instance(){ static Engine g; return g; }
Engine::Engine():impl(std::make_unique<Impl>()){}
Engine::~Engine()=default;

void Engine::init(){
  impl->scene.Build();
  impl->game.Init();
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
