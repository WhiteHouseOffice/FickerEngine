#include "game/physics/PhysicsWorldPBD.h"

#include <algorithm>

namespace fe {

static inline float dot3(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float len2(const Vec3& v) { return dot3(v,v); }

RigidClusterBody& PhysicsWorldPBD::createBodyFromProxy(const ColliderProxy& proxy, float massPerParticle) {
  RigidClusterBody b;
  b.id = m_nextId++;
  b.proxy = proxy;

  const int n = (int)proxy.spheres.size();
  b.particles.resize(n);
  b.radii.resize(n);

  const float invMass = (massPerParticle > 0.f) ? (1.f / massPerParticle) : 0.f;

  for (int i = 0; i < n; ++i) {
    b.particles[i].pos  = proxy.spheres[i].center;
    b.particles[i].prev = proxy.spheres[i].center;
    b.particles[i].invMass = invMass;
    b.radii[i] = proxy.spheres[i].radius;
  }

  // Constraints: connect all pairs for now (simple, stiff for small N=8).
  // Later we’ll switch to edges+diagonals only, but this is robust and easy.
  for (int i = 0; i < n; ++i) for (int j = i+1; j < n; ++j) {
    DistanceConstraint c;
    c.a = i; c.b = j;
    const Vec3 d = b.particles[j].pos - b.particles[i].pos;
    c.rest = std::sqrt(len2(d));
    b.constraints.push_back(c);
  }

  m_bodies.push_back(b);
  return m_bodies.back();
}

void PhysicsWorldPBD::step(float dt) {
  if (dt <= 0.f) return;

  m_accum += dt;

  int steps = 0;
  while (m_accum >= fixedDt && steps < maxSubsteps) {
    substep(fixedDt);
    m_accum -= fixedDt;
    steps++;
  }
}

void PhysicsWorldPBD::substep(float h) {
  // Integrate
  for (auto& b : m_bodies) {
    if (b.asleep) continue;
    integrate(b, h);
  }

  // Solve collisions + constraints
  for (int it = 0; it < solverIters; ++it) {
    // Ground
    if (enableGround) {
      for (auto& b : m_bodies) {
        if (b.asleep) continue;
        solveGround(b);
      }
    }

    // Body-body (naive O(M^2), ok for now)
    for (size_t i = 0; i < m_bodies.size(); ++i) {
      for (size_t j = i+1; j < m_bodies.size(); ++j) {
        if (m_bodies[i].asleep && m_bodies[j].asleep) continue;
        solveBodyBody(m_bodies[i], m_bodies[j]);
      }
    }

    // Constraints
    for (auto& b : m_bodies) {
      if (b.asleep) continue;
      solveConstraints(b);
    }
  }

  // Sleeping update
  for (auto& b : m_bodies) {
    updateSleeping(b, h);
  }
}

void PhysicsWorldPBD::integrate(RigidClusterBody& b, float h) {
  const Vec3 g = b.useGravity ? gravity : Vec3(0.f,0.f,0.f);

  for (auto& p : b.particles) {
    if (p.invMass == 0.f) continue;

    // Verlet integration: v = pos - prev
    Vec3 v = p.pos - p.prev;
    p.prev = p.pos;

    // Apply gravity
    p.pos = p.pos + v + (g * (h*h));
  }
}

void PhysicsWorldPBD::solveConstraints(RigidClusterBody& b) {
  for (const auto& c : b.constraints) {
    Particle& pa = b.particles[c.a];
    Particle& pb = b.particles[c.b];

    const float wA = pa.invMass;
    const float wB = pb.invMass;
    const float wSum = wA + wB;
    if (wSum <= 0.f) continue;

    const Vec3 d = pb.pos - pa.pos;
    const float d2 = len2(d);
    if (d2 <= 1e-12f) continue;

    const float dist = std::sqrt(d2);
    const float diff = (dist - c.rest) / dist;

    // Position correction (PBD)
    const Vec3 corr = d * diff;
    pa.pos = pa.pos + corr * ( wA / wSum);
    pb.pos = pb.pos - corr * ( wB / wSum);
  }
}

void PhysicsWorldPBD::solveGround(RigidClusterBody& b) {
  for (size_t i = 0; i < b.particles.size(); ++i) {
    Particle& p = b.particles[i];
    if (p.invMass == 0.f) continue;

    const float r = b.radii[i];
    const float bottom = p.pos.y - r;
    if (bottom < groundY) {
      p.pos.y += (groundY - bottom);
    }
  }
}

void PhysicsWorldPBD::solveSphereSphere(Particle& pa, float ra, Particle& pb, float rb) {
  const Vec3 d = pb.pos - pa.pos;
  const float d2 = len2(d);
  const float r = ra + rb;

  if (d2 >= r*r || d2 <= 1e-12f) return;

  const float dist = std::sqrt(d2);
  const Vec3 n = d * (1.f / dist);
  const float pen = (r - dist);

  const float wA = pa.invMass;
  const float wB = pb.invMass;
  const float wSum = wA + wB;
  if (wSum <= 0.f) return;

  // Split correction
  pa.pos = pa.pos - n * (pen * (wA / wSum));
  pb.pos = pb.pos + n * (pen * (wB / wSum));
}

void PhysicsWorldPBD::solveBodyBody(RigidClusterBody& a, RigidClusterBody& b) {
  // Naive all-pairs particle collisions (fine for 8 spheres).
  for (size_t i = 0; i < a.particles.size(); ++i) {
    for (size_t j = 0; j < b.particles.size(); ++j) {
      solveSphereSphere(a.particles[i], a.radii[i], b.particles[j], b.radii[j]);
    }
  }
}

bool PhysicsWorldPBD::solvePlayerSphere(Vec3& playerCenter, float playerRadius) {
  bool grounded = false;

  // Treat player as invMass=0 kinematic particle
  Particle player;
  player.pos = playerCenter;
  player.prev = playerCenter;
  player.invMass = 0.f;

  for (auto& b : m_bodies) {
    // if player interacts, wake body
    bool touched = false;

    for (size_t i = 0; i < b.particles.size(); ++i) {
      Particle& p = b.particles[i];
      const float r = b.radii[i];

      // Resolve sphere-sphere (player vs particle), but only move particle and player center.
      Vec3 d = p.pos - player.pos;
      float d2 = len2(d);
      float rr = (playerRadius + r);
      if (d2 >= rr*rr || d2 <= 1e-12f) continue;

      touched = true;

      float dist = std::sqrt(d2);
      Vec3 n = d * (1.f / dist);
      float pen = (rr - dist);

      // Move dynamic particle away; also move player opposite direction a bit so player can stand.
      if (p.invMass > 0.f) {
        p.pos = p.pos + n * pen;
      }
      player.pos = player.pos - n * pen;

      // Grounded if contact normal points up against player (player pushed upward)
      if (n.y > 0.5f) grounded = true;
    }

    if (touched) b.wake();
  }

  playerCenter = player.pos;
  return grounded;
}

bool PhysicsWorldPBD::updateSleeping(RigidClusterBody& b, float h) {
  if (b.particles.empty()) return false;

  // Estimate max particle velocity
  float vmax2 = 0.f;
  for (const auto& p : b.particles) {
    Vec3 v = (p.pos - p.prev) * (1.f / h);
    float v2 = len2(v);
    if (v2 > vmax2) vmax2 = v2;
  }

  if (vmax2 < b.sleepVel * b.sleepVel) {
    b.sleepTimer += h;
    if (b.sleepTimer >= b.sleepTime) {
      b.asleep = true;
      // snap prev to pos to avoid “wake jitter”
      for (auto& p : b.particles) p.prev = p.pos;
    }
  } else {
    b.sleepTimer = 0.f;
    b.asleep = false;
  }

  return b.asleep;
}

} // namespace fe
