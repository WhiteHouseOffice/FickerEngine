#include "game/Scene.h"
#include "game/GameObject.h"

void Scene::AddObject(const std::shared_ptr<GameObject>& obj) {
  objects.push_back(obj);
}

void Scene::Update(float /*dt*/) {
  // No-op for now (kept minimal; avoids extra deps)
}

void Scene::Draw() {
  // Intentionally empty until render pass & pipeline are wired
}
