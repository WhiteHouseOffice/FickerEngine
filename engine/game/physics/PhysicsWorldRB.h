#pragma once
#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "game/physics/RigidBody3D.h"

namespace fe {

struct Contact {
  uint32_t a = 0;
  uint32_t b = 0; // 0 => static world (ground/terrain/static meshes)

  Vec3 point{0.f,0.f,0.f};   // world contact point
  // Normal points from static/other toward A.
  Vec3 normal{0.f,1.f,0.f};
  float penetration = 0.f;   // >= 0
};

// Full-vertex static triangle meshes (Option B)
// Scene builds them from render/geom meshes; physics only queries them.
struct StaticTriMesh {
  std::vector<Vec3> verts;         // world-space vertices
  std::vector<uint32_t> indices;   // triangles (3 indices)
};

class PhysicsWorldRB {
public:
  Vec3 gravity{0.f, -18.f, 0.f};

  // Legacy infinite plane (usually disabled once terrain is enabled)
  bool  enableGround = true;
  float groundY = 0.f;

  // Terrain collision from render mesh data via callbacks.
  using TerrainHeightFn = float(*)(void* user, float x, float z);
  using TerrainNormalFn = Vec3 (*)(void* user, float x, float z);

  bool enableTerrain = false;

  void setTerrainCallbacks(TerrainHeightFn heightFn, TerrainNormalFn normalFn, void* user) {
    m_terrainHeightFn = heightFn;
    m_terrainNormalFn = normalFn;
    m_terrainUser = user;
    enableTerrain = (m_terrainHeightFn != nullptr);
  }

  // Static triangle meshes (platforms, future complex shapes)
  void setStaticMeshes(const std::vector<StaticTriMesh>& meshes) { m_staticMeshes = meshes; }

  float restitution = 0.0f;
  float friction = 0.6f;

  // Keeps contacts alive slightly above surfaces so friction can act at rest
  float contactSkin = 0.015f;

  float fixedDt = 1.f/120.f;
  int   maxSubsteps = 8;

  int velocityIters = 10;
  int positionIters = 3;

  uint32_t createBox(const Vec3& pos, const Vec3& halfExtents, float mass);
  RigidBoxBody* get(uint32_t id);

  void clearDynamics();

  // Player as kinematic sphere.
  bool collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded);

  void step(float dt);

  std::vector<RigidBoxBody>& bodiesMutable() { return m_bodies; }
  const std::vector<RigidBoxBody>& bodies() const { return m_bodies; }

private:
  float m_accum = 0.f;
  uint32_t m_nextId = 1;

  std::vector<RigidBoxBody> m_bodies;

  std::vector<StaticTriMesh> m_staticMeshes;

  TerrainHeightFn m_terrainHeightFn = nullptr;
  TerrainNormalFn m_terrainNormalFn = nullptr;
  void* m_terrainUser = nullptr;

  void substep(float h);

  void integrate(RigidBoxBody& b, float h);
  void applyDamping(RigidBoxBody& b, float h);
  void integrateOrientation(RigidBoxBody& b, float h);

  void gatherContacts(std::vector<Contact>& out);

  void contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out);
  void contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out);
  void contactsBoxTerrain(const RigidBoxBody& A, std::vector<Contact>& out);

  // Option B: box vs static triangle meshes
  void contactsBoxStaticMeshes(const RigidBoxBody& A, std::vector<Contact>& out);

  float terrainHeightAt(float x, float z) const;
  Vec3  terrainNormalAt(float x, float z) const;

  void solveVelocity(const Contact& c);
  void solvePosition(const Contact& c);

  Mat3 invInertiaWorld(const RigidBoxBody& b) const;
  void applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r);

  void updateSleeping(RigidBoxBody& b, float h, bool supported);
};

} // namespace fe
