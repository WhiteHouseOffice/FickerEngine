
#include "Engine.h"
#include "../render/IRenderer.h"
#include <memory>

struct Engine::Impl {
  EngineConfig cfg;
  std::unique_ptr<IRenderer> renderer;
  double time = 0.0;
  Impl(const EngineConfig& c, std::unique_ptr<IRenderer> r)
    : cfg(c), renderer(std::move(r)) {}
};

Engine::Engine(const EngineConfig& cfg, std::unique_ptr<IRenderer> r)
  : impl(std::make_unique<Impl>(cfg, std::move(r))) {}

int Engine::runOnce(double dt) {
  impl->time += dt;
  impl->renderer->beginFrame();
  draw(impl->time);
  impl->renderer->endFrame();
  return 0;
}

void Engine::draw(double t) {
  impl->renderer->drawTestScene(t);
}
