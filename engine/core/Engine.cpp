#include "core/Engine.h"
#include "game/Game.h"
#include "game/Scene.h"
#include "render/WebGPUContext.h"
#include "core/Time.h"

#include <cstdio>

using render::WebGPUContext;

Engine& Engine::instance() {
  static Engine s_engine;
  return s_engine;
}

void Engine::init() {
  std::puts("[Engine] init");
  Time::init();

  WebGPUContext::Get().init();
  WebGPUContext::Get().configure(1280, 720);

  scene = std::make_unique<Scene>();
  game  = std::make_unique<Game>();

  game->init();
  scene->buildDefaultScene(); // uses GridPlane / MarkerCross on the CPU side
}

void Engine::update() {
  Time::update();
  float dt = Time::deltaTime();

  if (game) {
    game->update(dt);
  }

  if (scene) {
    scene->update(dt);
  }
}

void Engine::render() {
  auto& ctx = WebGPUContext::Get();
  if (!ctx.isReady()) {
    return; // nothing to draw yet
  }

  auto backbuffer = ctx.acquireSwapchainView();
  // Rendering is stubbed out for now; once WebGPU is wired, Scene/Game
  // will use RenderMesh data to draw.
  ctx.present(backbuffer);
}

void Engine::shutdown() {
  std::puts("[Engine] shutdown");
  scene.reset();
  game.reset();
}

void Engine::stepOnce() {
  if (!initialized) {
    init();
    initialized = true;
  }

  update();
  render();
}
