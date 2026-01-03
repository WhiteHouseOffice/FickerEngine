#pragma once

#include "math/MiniMath.h"
#include "game/physics/RigidBody.h"

// Minimal gameplay object:
// - transform
// - optional physics (RigidBody)
// - optional AABB box collider (half extents)
class GameObject {
public:
  GameObject() = default;
  virtual ~GameObject() = default;

  // Simple stub: derived classes can override, or we flesh this out later.
  virtual void update(float dt);

  // Transform (minimal on purpose)
  Vec3 position{0.f, 0.f, 0.f};
  float yaw{0.f};

  // ---- Physics (optional) ----
  void enablePhysics(float mass, bool gravity = true);
  void disablePhysics();

  RigidBody* rigidBody();
  const RigidBody* rigidBody() const;

  void setVelocity(const Vec3& v);
  void addForce(const Vec3& f);

  // ---- Collider (optional) ----
  void enableBoxCollider(const Vec3& halfExtents);
  void disableBoxCollider();

  bool hasBoxCollider() const { return m_hasBoxCollider; }
  const Vec3& boxHalfExtents() const { return m_boxHalfExtents; }

private:
  bool m_hasRigidBody = false;
  RigidBody m_rigidBody;

  bool m_hasBoxCollider = false;
  Vec3 m_boxHalfExtents{0.5f, 0.5f, 0.5f};
};
