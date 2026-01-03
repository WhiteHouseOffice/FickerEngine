#pragma once

#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "game/physics/ColliderProxy.h"

namespace fe {

// Particle used for PBD/Verlet.
struct Particle {
  Vec3  pos{0.f,0.f,0.f};
  Vec3  prev{0.f,0.f,0.f};
  float invMass = 0.f; // 0 = locked/static
};

// Distance constraint between two particles.
struct DistanceConstraint {
  int   a = 0;
  int   b = 0;
  float rest = 0.f;
};

// A “rigid-ish” body represented by particles + constraints.
// For the 8-sphere cube: 8 particles + edge/diag constraints.
struct RigidClusterBody {
  uint32_t id = 0;

  // Proxy spheres define collision radii and initial layout.
  ColliderProxy proxy;

  std::vector<Particle> particles;
  std::vector<float>    radii;       // per particle
  std::vector<DistanceConstraint> constraints;

  bool useGravity = true;

  // Sleeping
  bool  asleep = false;
  float sleepTimer = 0.f;

  // Simple thresholds
  float sleepVel = 0.05f;      // units/sec
  float sleepTime = 0.5f;      // seconds below threshold => sleep

  // Derived transform (for rendering/debug)
  Vec3 centerOfMass() const;

  void wake();
};

} // namespace fe
