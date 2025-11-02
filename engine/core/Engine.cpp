#include "core/Engine.h"
#include "render/WebGPUContext.h"

using render::WebGPUContext;

Engine& Engine::instance() {
  static Engine s;
  return s;
}

void Engine::init() {
  // Bring up WebGPU (context methods are currently stubs; that’s fine)
  WebGPUContext::Get().Init();
  WebGPUContext::Get().Configure(1280, 720);
}

void Engine::update() {
  // Game / scene update would go here (kept empty to avoid extra deps)
}

void Engine::render() {
  auto& ctx = WebGPUContext::Get();

  // If Init() is still a stub (device/queue null), skip rendering safely.
  if (!ctx.Device() || !ctx.Queue()) return;

  // Acquire backbuffer (stub returns nullptr for now; still safe)
  WGPUTextureView backbuffer = ctx.BeginFrame();

  // TODO: when your pipeline is ready, encode a pass and draw here.

  // Present
  ctx.EndFrame(backbuffer);
}

void Engine::shutdown() {
  // Add explicit releases once WebGPUContext tracks resources
}

void Engine::stepOnce() {
  update();
  render();
}
