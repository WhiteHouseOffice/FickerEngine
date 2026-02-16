#pragma once

#include <memory>
#include <vector>

#include "math/MiniMath.h"
#include "game/GameObject.h"
#include "game/physics/PhysicsWorldRB.h"
#include "geom/TerrainGrid.h"

class Scene {
public:
  void init();
  void update(float dt);
  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

  void setPlayerSphere(const Vec3& center, float radius, const Vec3& velocity);
  bool getPlayerSphere(Vec3& outCenter, Vec3& outVelocity, bool& outGrounded) const;

private:
  float m_accumDt = 0.0f;
  bool  m_spawnCratesPending = false;
  float m_spawnTimer = 0.0f;

  std::vector<std::unique_ptr<GameObject>> m_objects;

  // Rigid-body physics for dynamic props (boxes)
  fe::PhysicsWorldRB m_rb;

  // Static colliders (built from GameObjects with box colliders)
  std::vector<fe::AABB> m_static;

  // Terrain mesh (render + physics sampling; full vertex count)
  engine::geom::TerrainGrid m_terrain;

  // Cached player proxy (fed into physics each frame)
  bool  m_playerValid = false;
  Vec3  m_playerCenter{0.f, 0.f, 0.f};
  float m_playerRadius = 0.5f;
  Vec3  m_playerVel{0.f,0.f,0.f};

  // Results after physics step
  Vec3  m_playerCenterOut{0.f,0.f,0.f};
  Vec3  m_playerVelOut{0.f,0.f,0.f};
  bool  m_playerGroundedOut = false;

  GameObject* createObject();

  void rebuildStaticAABBs();
};
