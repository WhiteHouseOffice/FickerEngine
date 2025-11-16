#include <cstdio>

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "render/RenderMesh.h"
#include "game/Game.h"
#include "game/Scene.h"
#include "core/Time.h"
#include "core/Engine.h"


Engine& Engine::instance() {
  static Engine s_instance;
  return s_instance;
}

void Engine::init() {
  if (initialized) return;

  scene = std::make_unique<Scene>();
  game  = std::make_unique<Game>();

  game->init();
  // Scene constructor already builds grid + marker.
  initialized = true;
}

void Engine::update() {
  if (!initialized) return;

  const float dt = Time::deltaTime();

  if (game)  game->update(dt);
  if (scene) scene->update(dt);
}

// UPDATED: Step 2+ render
void Engine::render() {
  // Temporary CPU-only debug "renderer".
  // We generate simple geometry once and then just log its stats each frame.

  static bool          s_initialized = false;
  static geom::GridPlane   s_grid;
  static geom::MarkerCross s_marker;
  static RenderMesh        s_gridMesh;
  static RenderMesh        s_markerMesh;

  if (!s_initialized) {
    // Upload CPU geometry into our CPU-only RenderMesh.
    s_gridMesh.uploadGrid(s_grid);
    s_markerMesh.uploadMarker(s_marker);
    s_initialized = true;
  }

  // Step 2: log mesh stats every frame so we can see the pipeline is alive.
  s_gridMesh.debugPrint("grid");
  s_markerMesh.debugPrint("marker");

  // Later this is where we’ll:
  //  - build view/projection from Game::view()
  //  - walk Scene, upload objects
  //  - forward to WebGPU/WebGL backend
}


void Engine::shutdown() {
  if (!initialized) return;

  scene.reset();
  game.reset();
  initialized = false;
}
