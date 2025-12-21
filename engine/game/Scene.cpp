#include "game/Scene.h"

void Scene::init() {
  // If you already have a different method that sets up objects,
  // call it here (or keep empty for now).
}

// If you already have camera or input-driven state, update it here.
void Scene::update(float /*dt*/) {
  // TODO: camera movement / physics later
}

void Scene::render(const Mat4& view, const Mat4& proj) {
    (void)view;
    (void)proj;

    // draw stuff (same code as before)
}


void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // TODO: draw grid / marker debug here
}
