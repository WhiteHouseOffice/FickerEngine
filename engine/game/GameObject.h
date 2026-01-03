#pragma once

#include "math/MiniMath.h"

// Minimal gameplay object used primarily for static world/platform colliders.
// (Dynamic physics props live in PhysicsWorldRB for now.)
class GameObject {
public:
  GameObject() = default;
  virtual ~GameObject() = default;

  virtual void update(float dt);

  // Transform
  Vec3 position{0.f, 0.f, 0.f};
  float yaw{0.f};

  // ---- Static AABB collider (optional) ----
  void enableBoxCollider(const Vec3& halfExtents);
  void disableBoxCollider();

  bool hasBoxCollider() const { return m_hasBoxCollider; }
  const Vec3& boxHalfExtents() const { return m_boxHalfExtents; }

private:
  bool m_hasBoxCollider = false;
  Vec3 m_boxHalfExtents{0.5f, 0.5f, 0.5f};
};
