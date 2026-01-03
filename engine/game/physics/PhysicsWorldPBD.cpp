#include "game/physics/PhysicsWorldPBD.h"

#include <algorithm>
#include <cmath>

namespace fe {

static inline float dot3(const Vec3& a, const Vec3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline float len2(const Vec3& v) { return dot3(v,v); }

RigidClusterBody& PhysicsWorldPBD::createBodyFromProxy(
  const ColliderProxy& proxy,
  float massPerParticle
) {
  RigidClusterBody b;
  b.id = m_nextId++;
  b.proxy = proxy;

  const int n = (int)proxy.spheres.size();
  b.particles.resize(n);
  b.radii.resize(n);

  const float invMass =
    (massPerParticle > 0.f) ? (1.f / massPerParticle) : 0.f;

  for (int i = 0; i < n; ++i) {
    b.particles[i].pos  = proxy.spheres[i].center;
    b.particles[i].prev = proxy.spheres[i].center;
    b.particles[i].invMass = invMass;
    b.radii[i] = proxy.spheres[i].radius;
  }

  // Rigid constraints: connect all pairs (OK for small N like 8)
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      DistanceConstraint c;
      c.a = i;
      c.b = j;
      Vec3 d = b.particles[j].pos - b.particles[i].pos;
      c.rest = std::sqrt(len2(d));
      b.constraints.push_back(c);
    }
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

    // Multiple collision passes reduce squish
    for (int cit = 0; cit < collisionIters; ++cit) {

      if (enableGround) {
        for (auto& b : m_bodies) {
          if (b.asleep) continue;
          solveGround(b);
        }
      }

      for (size_t i = 0; i < m_bodies.size(); ++i) {
        for (size_t j = i + 1; j < m_bodies.size(); ++j) {
          if (m_bodies[i].asleep && m_bodies[j].asleep) continue;
          solveBodyBody(m_bodies[i], m_bodies[j]);
        }
      }
    }

    for (auto& b : m_bodies) {
      if (b.asleep) continue;
      solveConstraints(b);
    }
  }

  for (auto& b : m_bodies) {
    updateSleeping(b, h);
  }
}

void PhysicsWorldPBD::integrate(RigidClusterBody& b, float h) {
  const Vec3 g = b.useGravity ? gravity : Vec3(0.f,0.f,0.f);
  const float damping = 0.98f;

  for (auto& p : b.particles) {
    if (p.invMass == 0.f) continue;

    Vec3 v = p.pos - p.prev;
    v = v * damping;

    p.prev = p.pos;
    p.pos  = p.pos + v + (g * (h * h));
  }
}

void PhysicsWorldPBD::solveConstraints(RigidClusterBody& b) {
  for (const auto& c : b.constraints) {
    Particle& pa = b.particles[c.a];
    Particle& pb = b.particles[c.b];

    float wA = pa.invMass;
    float wB = pb.invMass;
    float wSum = wA + wB;
    if (wSum <= 0.f) continue;

    Vec3 d = pb.pos - pa.pos;
    float d2 = len2(d);
    if (d2 <= 1e-12f) continue;

    float dist = std::sqrt(d2);
    float diff = (dist - c.rest) / dist;
    Vec3 corr = d * diff;

    pa.pos = pa.pos + corr * ( wA / wSum);
    pb.pos = pb.pos - corr * ( wB / wSum);
  }
}

void PhysicsWorldPBD::solveGround(RigidClusterBody& b) {
  const float groundFriction = 0.60f;

  for (size_t i = 0; i < b.particles.size(); ++i) {
    Particle& p = b.particles[i];
    if (p.invMass == 0.f) continue;

    float r = b.radii[i];
    float bottom = p.pos.y - r;

    if (bottom < groundY) {
      p.pos.y += (groundY - bottom);

      Vec3 v = p.pos - p.prev;
      v.x *= (1.0f - groundFriction);
      v.z *= (1.0f - groundFriction);
      p.prev = p.pos - v;
    }
  }
}

void PhysicsWorldPBD::solveSphereSphere(
  Particle& pa, float ra,
  Particle& pb, float rb
) {
  Vec3 d = pb.pos - pa.pos;
  float d2 = len2(d);
  float r = ra + rb;

  if (d2 >= r*r || d2 <= 1e-12f) return;

  float dist = std::sqrt(d2);
  Vec3 n = d * (1.f / dist);
  float pen = (r - dist);

  float wA = pa.invMass;
  float wB = pb.invMass;
  float wSum = wA + wB;
  if (wSum <= 0.f) return;

  pa.pos = pa.pos - n * (pen * (wA / wSum));
  pb.pos = pb.pos + n * (pen * (wB / wSum));
}

void PhysicsWorldPBD::solveBodyBody(
  RigidClusterBody& a,
  RigidClusterBody& b
) {
  for (size_t i = 0; i < a.particles.size(); ++i) {
    for (size_t j = 0; j < b.particles.size(); ++j) {
      solveSphereSphere(
        a.particles[i], a.radii[i],
        b.particles[j], b.radii[j]
      );
    }
  }
}

bool PhysicsWorldPBD::updateSleeping(RigidClusterBody& b, float h) {
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
      for (auto& p : b.particles) p.prev = p.pos;
    }
  } else {
    b.sleepTimer = 0.f;
    b.asleep = false;
  }

  return b.asleep;
}

bool PhysicsWorldPBD::solvePlayerSphere(Vec3&, float) {
  return false;
}

} // namespace fe
