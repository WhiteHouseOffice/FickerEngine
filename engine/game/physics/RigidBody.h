#pragma once

#include "math/MiniMath.h"

// Ultra-light rigid body for simple, scalable physics.
// - Linear motion only (no rotation physics yet)
// - invMass == 0 means static body
struct RigidBody {
  float invMass = 0.0f;

  Vec3 velocity{0.f, 0.f, 0.f};

  bool useGravity = true;

  // Per-second damping to keep simulation stable
  float linearDamping = 0.10f;

  // Optional force accumulator (cleared every frame)
  Vec3 forceAccum{0.f, 0.f, 0.f};

  bool isDynamic() const { return invMass > 0.0f; }

  void setMass(float mass) {
    invMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
  }

  void addForce(const Vec3& f) { forceAccum = forceAccum + f; }
  void clearForces() { forceAccum = Vec3(0.f, 0.f, 0.f); }
};
