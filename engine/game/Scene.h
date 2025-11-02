#pragma once
#include <memory>
#include <vector>

class GameObject;

class Scene {
public:
  void AddObject(const std::shared_ptr<GameObject>& obj);
  void Update(float dt);
  void Draw();

private:
  std::vector<std::shared_ptr<GameObject>> objects;
};
