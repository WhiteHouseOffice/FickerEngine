#include <cstdio>

#include "game/Game.h"
#include "game/Scene.h"
#include "core/Time.h"
#include "core/Engine.h"
#include "math/MiniMath.h"  // Mat4, perspective()

Engine& Engine::instance() {
  static Engine s_instance;
  return s_instance;
}

void Engine::init() {
  if (initialized) return;

  scene = std::make_unique<Scene>();
  game  = std::make_unique<Game>();

  if (game)  game->init();
  if (scene) scene->init();

  initialized = true;
}

void Engine::update() {
  if (!initialized) return;

  const float dt = Time::deltaTime();

  if (game)  game->update(dt);
  if (scene) scene->update(dt);
}

void Engine::render() {
  if (!initialized) return;

  // Build a simple camera from Game and a fixed projection.
  Mat4 view = Mat4::identity();
  if (game) {
    view = game->view();
  }

  const float fovRad = 60.0f * 3.14159265f / 180.0f;
  const float aspect = 4.0f / 3.0f;  // TODO: replace with real viewport aspect
  Mat4 proj = perspective(fovRad, aspect, 0.1f, 100.0f);

  if (scene) {
    scene->render(view, proj);
    scene->renderDebug(view, proj);
  }
}

void Engine::stepOnce() {
  if (!initialized) {
    return;
  }

  update();
  render();
}

void Engine::shutdown() {
  if (!initialized) return;

  scene.reset();
  game.reset();
  initialized = false;
}
