#pragma once
#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "game/physics/RigidBody3D.h"

namespace fe {

struct Contact {
  uint32_t a = 0;
  uint32_t b = 0; // 0 => static world (AABB/ground/terrain)

  Vec3 point{0.f,0.f,0.f};   // world contact point
  // IMPORTANT: normal points from "other" (B or static) toward A.
  // This matches solver convention (impulse applied to A along +n).
  Vec3 normal{0.f,1.f,0.f};
  float penetration = 0.f;   // >=0. Can be 0 inside contactSkin band.
};

class PhysicsWorldRB {
public:
  Vec3 gravity{0.f, -18.f, 0.f};

  // Legacy infinite plane (you can disable this when using terrain mesh collision)
  bool  enableGround = true;
  float groundY = 0.f;

  // Terrain collision from render mesh data (full vertex count) via callbacks.
  // Scene owns the mesh; physics only queries heights/normals.
  using TerrainHeightFn = float(*)(void* user, float x, float z);
  using TerrainNormalFn = Vec3 (*)(void* user, float x, float z);

  bool enableTerrain = false;

  void setTerrainCallbacks(TerrainHeightFn heightFn, TerrainNormalFn normalFn, void* user) {
    m_terrainHeightFn = heightFn;
    m_terrainNormalFn = normalFn;
    m_terrainUser = user;
    enableTerrain = (m_terrainHeightFn != nullptr);
  }

  float restitution = 0.0f;
  float friction = 0.6f;

  // Contact skin keeps contacts alive slightly above surfaces, so friction works at rest
  float contactSkin = 0.06f;

  float fixedDt = 1.f/120.f;
  int   maxSubsteps = 8;

  int velocityIters = 10;
  int positionIters = 3;

  uint32_t createBox(const Vec3& pos, const Vec3& halfExtents, float mass);
  RigidBoxBody* get(uint32_t id);

  void clearDynamics();

  // Static colliders (platforms). Keeping AABBs for now — terrain uses full mesh vertices.
  void setStaticAABBs(const std::vector<AABB>& aabbs) { m_static = aabbs; }

  // Player as kinematic sphere.
  bool collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded);

  void step(float dt);

  std::vector<RigidBoxBody>& bodiesMutable() { return m_bodies; }
  const std::vector<RigidBoxBody>& bodies() const { return m_bodies; }

private:
  float m_accum = 0.f;
  uint32_t m_nextId = 1;

  std::vector<RigidBoxBody> m_bodies;
  std::vector<AABB> m_static;

  TerrainHeightFn m_terrainHeightFn = nullptr;
  TerrainNormalFn m_terrainNormalFn = nullptr;
  void* m_terrainUser = nullptr;

  void substep(float h);

  void integrate(RigidBoxBody& b, float h);
  void applyDamping(RigidBoxBody& b, float h);
  void integrateOrientation(RigidBoxBody& b, float h);

  void gatherContacts(std::vector<Contact>& out);
  void contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out);
  void contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out);
  void contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out);
  void contactsBoxTerrain(const RigidBoxBody& A, std::vector<Contact>& out);

  float terrainHeightAt(float x, float z) const;
  Vec3  terrainNormalAt(float x, float z) const;

  void solveVelocity(const Contact& c);
  void solvePosition(const Contact& c);

  Mat3 invInertiaWorld(const RigidBoxBody& b) const;
  void applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r);

  void updateSleeping(RigidBoxBody& b, float h, bool supported);
};

} // namespace fe
