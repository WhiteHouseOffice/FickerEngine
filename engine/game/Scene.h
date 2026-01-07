#pragma once

#include <memory>
#include <vector>

#include "math/MiniMath.h"
#include "game/GameObject.h"
#include "game/physics/PhysicsWorldRB.h"

class Scene {
public:
  void init();
  void update(float dt);
// Rendering split is intentional:
// - render(): real game rendering (RenderMesh, triangles, shipping visuals)
// - renderDebug(): developer-only overlays (grid, colliders, physics gizmos)
//
// render() may be empty during early engine bring-up.
// Do NOT move debug drawing into render().
  void render(const Mat4& view, const Mat4& proj);
  void renderDebug(const Mat4& view, const Mat4& proj);

  // Player proxy (kinematic sphere) for pushing / standing on props.
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
