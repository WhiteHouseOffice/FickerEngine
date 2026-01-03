#pragma once

#include <vector>
#include <cstdint>

#include "math/MiniMath.h"
#include "game/physics/RigidClusterBody.h"

namespace fe {

class PhysicsWorldPBD {
public:
  Vec3 gravity{0.f, -18.0f, 0.f};

  bool  enableGround = true;
  float groundY = 0.f;

  // Fixed timestep substepping
  float fixedDt = 1.0f / 180.0f;
  int   maxSubsteps = 8;

  // Solver iterations (higher = stiffer bodies)
  int solverIters = 10;

  // Extra collision passes per substep
  int collisionIters = 6;

  RigidClusterBody& createBodyFromProxy(const ColliderProxy& proxy,
                                       float massPerParticle);

  void step(float dt);

  // Player interaction (not wired yet)
  bool solvePlayerSphere(Vec3& playerCenter, float playerRadius);

  const std::vector<RigidClusterBody>& bodies() const { return m_bodies; }
  std::vector<RigidClusterBody>&       bodies()       { return m_bodies; }

private:
  float m_accum = 0.f;
  uint32_t m_nextId = 1;

  std::vector<RigidClusterBody> m_bodies;

  void substep(float h);

  void integrate(RigidClusterBody& b, float h);
  void solveConstraints(RigidClusterBody& b);

  void solveGround(RigidClusterBody& b);
  void solveBodyBody(RigidClusterBody& a, RigidClusterBody& b);
  void solveSphereSphere(Particle& pa, float ra,
                         Particle& pb, float rb);

  bool updateSleeping(RigidClusterBody& b, float h);
};

} // namespace fe
