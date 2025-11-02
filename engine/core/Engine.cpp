#include "core/Engine.h"
#include "core/Time.h"
#include <cmath>
#include <memory>

#if defined(FE_WEB)
  #include <emscripten/emscripten.h>
  #include <emscripten/bind.h>
#endif

struct Engine::Impl {
  Time time;
  double accumulator = 0.0;
  double simTime = 0.0;     // seconds simulated
  float  angleDeg   = 0.0f; // demo "game state"

  void update_fixed(double dt) {
    simTime += dt;
    angleDeg += static_cast<float>(90.0 * dt); // rotate 90°/s
    if (angleDeg >= 360.f) angleDeg -= 360.f;
  }

  void render_interp(double /*alpha*/) {
    // Hook C++ WebGPU renderer here later.
  }
};

Engine& Engine::instance() {
  static Engine g;
  return g;
}

Engine::Engine() : impl(std::make_unique<Impl>()) {}
Engine::~Engine() = default;

void Engine::init() {
  // init subsystems / renderer later
}

void Engine::stepOnce() {
  double frameDt = impl->time.tick();
  impl->accumulator += frameDt;
  while (impl->accumulator >= Time::kFixedDt) {
    impl->update_fixed(Time::kFixedDt);
    impl->accumulator -= Time::kFixedDt;
  }
  const double alpha = impl->accumulator / Time::kFixedDt;
  impl->render_interp(alpha);
}

float Engine::angle() const { return impl->angleDeg; }

#if defined(FE_WEB)
extern "C" {
  EMSCRIPTEN_KEEPALIVE
  void fe_step_once() { Engine::instance().stepOnce(); }

  EMSCRIPTEN_KEEPALIVE
  float fe_get_angle() { return Engine::instance().angle(); }
}

EMSCRIPTEN_BINDINGS(fe_bindings) {
  emscripten::function("stepOnce", &fe_step_once);
  emscripten::function("getAngle", &fe_get_angle);
}
#endif
