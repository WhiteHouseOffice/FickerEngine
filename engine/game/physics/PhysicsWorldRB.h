#pragma once
#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "game/physics/RigidBody3D.h"

namespace fe {

struct Contact {
  uint32_t a = 0;
  uint32_t b = 0; // 0 => static world (AABB or ground)

  Vec3 point{0.f,0.f,0.f};   // world
  Vec3 normal{0.f,1.f,0.f};  // from A toward B (or out of static)
  float penetration = 0.f;

  // --- Warm-start cache ---
  // Accumulated impulses for this point carried across substeps.
  // (Not persistent across program runs; built every frame.)
  float accumNormalImpulse = 0.f;
  float accumTangentImpulse = 0.f;
};

class PhysicsWorldRB {
public:
  Vec3 gravity{0.f, -18.f, 0.f};

  bool  enableGround = true;
  float groundY = 0.f;

  float restitution = 0.0f;
  float friction = 0.6f;

  float fixedDt = 1.f/120.f;
  int   maxSubsteps = 8;

  int velocityIters = 10;
  int positionIters = 3;

  // --- Continuous collision detection (CCD) ---
  // Bounding-sphere sweep CCD. Eliminates most tunneling and the large
  // positional "teleport" corrections that happen after deep penetration.
  bool  enableCCD = true;
  float ccdRadiusScale = 1.05f; // >1 adds a small safety margin
  float ccdSlop = 0.0015f;      // push-out slop after TOI

  uint32_t createBox(const Vec3& pos, const Vec3& halfExtents, float mass);
  RigidBoxBody* get(uint32_t id);

  void clearDynamics();

  // Provide static colliders each frame (platforms from Scene/GameObject)
  void setStaticAABBs(const std::vector<AABB>& aabbs) { m_static = aabbs; }

  // Player as kinematic sphere. Returns true if we corrected it (collided).
  // If outGrounded != nullptr, set to true if supporting contact was found.
  bool collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded);

  void step(float dt);

  const std::vector<RigidBoxBody>& bodies() const { return m_bodies; }

private:
  float m_accum = 0.f;
  float m_contactDt = 0.f; // dt of current substep (for speculative contacts)
  uint32_t m_nextId = 1;

  std::vector<RigidBoxBody> m_bodies;
  std::vector<AABB> m_static;

  // Previous-substep contacts for warm-starting (simple point matching).
  // This reduces popping/teleporting because the solver starts from a
  // near-solved impulse state instead of zero every substep.
  std::vector<Contact> m_prevContacts;

  void substep(float h);

  void integrate(RigidBoxBody& b, float h, const Vec3& oldPos);
  void applyDamping(RigidBoxBody& b, float h);
  void integrateOrientation(RigidBoxBody& b, float h);

  // CCD helpers (bounding-sphere sweeps)
  bool sweepSphereVsAABB(const Vec3& p0, const Vec3& p1, float r, const AABB& box, float& outT, Vec3& outN) const;
  bool sweepSphereVsGround(const Vec3& p0, const Vec3& p1, float r, float& outT) const;

  void gatherContacts(std::vector<Contact>& out);
  void contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out);
  void contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out);
  void contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out);

  void solveVelocity(Contact& c);
  void solvePosition(const Contact& c);

  Mat3 invInertiaWorld(const RigidBoxBody& b) const;
  void applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r);
  void updateSleeping(RigidBoxBody& b, float h);
};

} // namespace fe
