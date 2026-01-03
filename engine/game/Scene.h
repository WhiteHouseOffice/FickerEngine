#pragma once

#include <memory>
#include <vector>

#include "math/MiniMath.h"
#include "game/GameObject.h"
#include "game/physics/PhysicsWorld.h"
#include "game/physics/PhysicsWorldPBD.h"

class Scene {
public:
  void init();
  void update(float dt);

  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

  // Provide a kinematic sphere proxy (usually the player) so it can push boxes.
  void setPlayerSphere(const Vec3& center, float radius);

private:
  std::vector<std::unique_ptr<GameObject>> m_objects;
  PhysicsWorld m_physics;
  fe::PhysicsWorldPBD m_pbd;

  // Cached proxy (fed into PhysicsWorld each frame)
  bool  m_playerSphereValid = false;
  Vec3  m_playerSphereCenter{0.f, 0.f, 0.f};
  float m_playerSphereRadius = 0.5f;

  GameObject* createObject();
};
