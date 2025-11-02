#include "core/Engine.h"
#include "render/WebGPUContext.h"

#include "game/Scene.h"
#include "game/Game.h"

// If you have these geometry helpers, keep the includes.
// Otherwise you can comment them out until we hook them up.
// #include "geom/GridPlane.h"
// #include "geom/MarkerCross.h"

Engine& Engine::instance() {
  static Engine e;
  return e;
}

void Engine::init() {
  // Bring up GPU
  render::WebGPUContext::Get().Init();
  render::WebGPUContext::Get().Configure(1280, 720);

  // Create scene + game logic containers if your headers declare them
  // (Safe to leave as nullptrs if you don’t need them yet.)
  // scene = std::make_unique<Scene>();
  // game  = std::make_unique<Game>();
}

void Engine::update() {
  // Advance gameplay / simulation; if you have a Time system, call it here.
  // if (game)  game->Update();
  // if (scene) scene->Update();
}

void Engine::render() {
  auto& ctx = render::WebGPUContext::Get();
  if (!ctx.Device() || !ctx.Queue()) {
    // If Init() is still stubbed, just early-out to avoid null deref.
    return;
  }

  WGPUTextureView backbuffer = ctx.BeginFrame();

  // Here you’d encode your render pass and draw calls.
  // This stub keeps the method compiled until we wire Dawn pass creation.

  ctx.EndFrame(backbuffer);
}

void Engine::shutdown() {
  // Place for cleanup when you add explicit releases in WebGPUContext
}
