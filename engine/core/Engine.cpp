#include "core/Engine.h"
#include "core/Time.h"
#include "render/WebGPUContext.h"

// Singleton instance
Engine& Engine::instance() {
  static Engine inst;
  return inst;
}

Engine::Engine()  = default;
Engine::~Engine() = default;

void Engine::init() {
  // Time system
  Time::init();

  // Stubbed WebGPU context (no real GPU calls yet)
  auto& ctx = render::WebGPUContext::Get();
  ctx.init();
  ctx.configure(1280, 720);

  // Game / camera
  game.init();
}

void Engine::stepOnce() {
  // Update timing
  Time::update();
  float dt = Time::deltaTime();

  // Update game logic (camera movement, etc.)
  game.update(dt);

  // Frame hooks (currently no real rendering)
  auto& ctx = render::WebGPUContext::Get();
  if (!ctx.isReady()) {
    return;
  }

  ctx.beginFrame();
  // TODO: once we have a real renderer, use game.view() here
  ctx.endFrame();
}
