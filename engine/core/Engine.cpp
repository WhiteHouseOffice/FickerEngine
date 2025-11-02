#include "core/Engine.h"
#include "engine/core/Time.h"
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

  // Your fixed-step game logic
  void update_fixed(double dt) {
    simTime += dt;
    angleDeg += static_cast<float>(90.0 * dt); // rotate 90°/s for demonstration
    if (angleDeg >= 360.f) angleDeg -= 360.f;
  }

  // Your render with interpolation (alpha in [0,1]); hook to renderer later
  void render_interp(double /*alpha*/) {
    // For now we don't drive GPU from C++ yet; JS fallback draws the triangle.
    // When the C++ WebGPU path is ready, call renderer->draw(angleDeg, alpha) here.
  }
};

Engine& Engine::instance() {
  static Engine g;
  return g;
}

Engine::Engine() : impl(std::make_unique<Impl>()) {}
Engine::~Engine() = default;

void Engine::init() {
  // Initialize subsystems/renderers here later.
}

void Engine::stepOnce() {
  // Variable frame dt
  double frameDt = impl->time.tick();
  impl->accumulator += frameDt;

  // Run 0..N fixed steps
  while (impl->accumulator >= Time::kFixedDt) {
    impl->update_fixed(Time::kFixedDt);
    impl->accumulator -= Time::kFixedDt;
  }

  // Interpolation factor for rendering (optional)
  const double alpha = impl->accumulator / Time::kFixedDt;
  impl->render_interp(alpha);
}

float Engine::angle() const { return impl->angleDeg; }

#if defined(FE_WEB)
// --- Web bindings so JS/CI can advance one frame ---
extern "C" {
  EMSCRIPTEN_KEEPALIVE
  void fe_step_once() { Engine::instance().stepOnce(); }

  EMSCRIPTEN_KEEPALIVE
  float fe_get_angle() { return Engine::instance().angle(); }
}

// Embind: provides Module.stepOnce() and Module.getAngle()
EMSCRIPTEN_BINDINGS(fe_bindings) {
  emscripten::function("stepOnce", &fe_step_once);
  emscripten::function("getAngle", &fe_get_angle);
}
#endif
