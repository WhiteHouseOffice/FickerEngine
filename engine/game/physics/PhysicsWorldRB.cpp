#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>

namespace fe {

// ------------------------------------------------------------
// helpers
// ------------------------------------------------------------
static inline float dot3(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float len2(const Vec3& v){ return dot3(v,v); }

static inline Vec3 clampVec3(const Vec3& v, const Vec3& mn, const Vec3& mx) {
  return Vec3(
    (v.x < mn.x) ? mn.x : (v.x > mx.x ? mx.x : v.x),
    (v.y < mn.y) ? mn.y : (v.y > mx.y ? mx.y : v.y),
    (v.z < mn.z) ? mn.z : (v.z > mx.z ? mx.z : v.z)
  );
}

// ------------------------------------------------------------
// creation / access
// ------------------------------------------------------------
uint32_t PhysicsWorldRB::createBox(const Vec3& pos, const Vec3& halfExtents, float mass) {
  RigidBoxBody b;
  b.id = m_nextId++;
  b.position = pos;
  b.orientation = Quat::identity();     // keep identity while angular is disabled
  b.halfExtents = halfExtents;

  if (mass > 0.f) {
    b.invMass = 1.f / mass;
    b.invInertiaLocal = boxInertiaInvLocal(halfExtents, mass);
  } else {
    b.invMass = 0.f;
    b.invInertiaLocal = Mat3{};
  }

  // IMPORTANT: keep them awake by default
  b.asleep = false;
  b.sleepTimer = 0.f;

  m_bodies.push_back(b);
  return b.id;
}

RigidBoxBody* PhysicsWorldRB::get(uint32_t id) {
  for (auto& b : m_bodies) if (b.id == id) return &b;
  return nullptr;
}

void PhysicsWorldRB::clearDynamics() {
  m_bodies.clear();
  m_nextId = 1;
  m_accum = 0.f;
}

// ------------------------------------------------------------
// integration (LINEAR ONLY)
// ------------------------------------------------------------
void PhysicsWorldRB::integrate(RigidBoxBody& b, float h) {
  if (!b.isDynamic() || b.asleep) return;

  if (b.useGravity) {
    b.linearVelocity = b.linearVelocity + gravity * h;
  }

  b.position = b.position + b.linearVelocity * h;

  // NOTE:
  // rotation intentionally disabled until solver is stable
  // b.orientation stays identity, b.angularVelocity ignored
}

// ------------------------------------------------------------
// contacts: ground only (for now)
// ------------------------------------------------------------
void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  // With rotation disabled, the "lowest vertex" is just center.y - halfExtents.y.
  float minY = A.position.y - A.halfExtents.y;
  if (minY < groundY) {
    Contact c;
    c.a = A.id;
    c.b = 0;
    c.point = Vec3(A.position.x, minY, A.position.z);
    c.normal = Vec3(0,1,0);
    c.penetration = (groundY - minY);
    out.push_back(c);
  }
}

void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();
  out.reserve(32);

  for (auto& b : m_bodies) {
    if (!b.isDynamic() || b.asleep) continue;
    if (enableGround) contactsBoxGround(b, out);
    // NOTE: static AABBs + box-box intentionally skipped for stability phase
  }
}

// ------------------------------------------------------------
// solver (LINEAR ONLY — SAFE)
// ------------------------------------------------------------
void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A || !A->isDynamic()) return;

  Vec3 n = c.normal;
  float n2 = dot3(n,n);
  if (n2 < 1e-8f) return;
  n = n * (1.f / std::sqrt(n2));

  float vn = dot3(A->linearVelocity, n);
  if (vn > 0.f) return;

  float denom = A->invMass;
  if (denom <= 0.f) return;

  float j = -(1.f + restitution) * vn / denom;
  if (!std::isfinite(j)) return;

  // apply impulse
  A->linearVelocity = A->linearVelocity + n * (j * A->invMass);

  // crude friction: damp lateral velocity when on ground contact
  Vec3 v = A->linearVelocity;
  Vec3 lateral(v.x, 0.f, v.z);
  float lat2 = len2(lateral);
  if (lat2 > 1e-12f) {
    float k = std::max(0.f, 1.f - friction * 0.6f);
    A->linearVelocity.x *= k;
    A->linearVelocity.z *= k;
  }

  A->asleep = false;
  A->sleepTimer = 0.f;
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A || !A->isDynamic()) return;

  const float slop = 0.002f;
  const float percent = 0.9f;

  float pen = std::max(0.f, c.penetration - slop);
  if (pen <= 0.f) return;

  Vec3 n = c.normal;
  float n2 = dot3(n,n);
  if (n2 < 1e-8f) return;
  n = n * (1.f / std::sqrt(n2));

  A->position = A->position + n * (percent * pen);

  // if we pushed up, kill downward velocity
  if (n.y > 0.5f && A->linearVelocity.y < 0.f) {
    A->linearVelocity.y = 0.f;
  }
}

// ------------------------------------------------------------
// sleeping
// ------------------------------------------------------------
void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h) {
  if (!b.isDynamic()) return;

  const float v2 = len2(b.linearVelocity);
  if (v2 < b.sleepVel*b.sleepVel) {
    b.sleepTimer += h;
    if (b.sleepTimer >= b.sleepTime) {
      b.asleep = true;
      b.linearVelocity = Vec3(0,0,0);
      b.angularVelocity = Vec3(0,0,0);
    }
  } else {
    b.sleepTimer = 0.f;
    b.asleep = false;
  }
}

// ------------------------------------------------------------
// player collision (KEEP IT, so link succeeds)
// ------------------------------------------------------------
bool PhysicsWorldRB::collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded) {
  bool hit = false;
  bool grounded = false;

  // collide vs dynamic boxes (AABB approximation since rotation is disabled)
  for (auto& b : m_bodies) {
    if (!b.isDynamic()) continue;

    Vec3 mn(b.position.x - b.halfExtents.x, b.position.y - b.halfExtents.y, b.position.z - b.halfExtents.z);
    Vec3 mx(b.position.x + b.halfExtents.x, b.position.y + b.halfExtents.y, b.position.z + b.halfExtents.z);

    Vec3 cl = clampVec3(center, mn, mx);
    Vec3 d = center - cl;
    float d2 = len2(d);
    if (d2 < radius*radius) {
      float dist = (d2 > 1e-12f) ? std::sqrt(d2) : 0.f;
      Vec3 n = (dist > 1e-6f) ? (d * (1.f/dist)) : Vec3(0,1,0);
      float pen = radius - dist;

      center = center + n * pen;

      float vn = dot3(playerVel, n);
      if (vn < 0.f) playerVel = playerVel - n * vn;

      if (n.y > 0.6f) grounded = true;
      hit = true;

      b.asleep = false;
      b.sleepTimer = 0.f;
    }
  }

  // collide vs static AABBs (kept)
  for (const auto& s : m_static) {
    Vec3 cl = clampVec3(center, s.min, s.max);
    Vec3 d = center - cl;
    float d2 = len2(d);
    if (d2 < radius*radius) {
      // push out along best axis (simple)
      float px = std::min(center.x - s.min.x, s.max.x - center.x);
      float py = std::min(center.y - s.min.y, s.max.y - center.y);
      float pz = std::min(center.z - s.min.z, s.max.z - center.z);

      Vec3 n(0,1,0);
      float pen = py;
      if (px < pen) { pen=px; n = (center.x < (s.min.x+s.max.x)*0.5f) ? Vec3(-1,0,0) : Vec3(1,0,0); }
      if (pz < pen) { pen=pz; n = (center.z < (s.min.z+s.max.z)*0.5f) ? Vec3(0,0,-1) : Vec3(0,0,1); }

      center = center + n * pen;

      float vn = dot3(playerVel, n);
      if (vn < 0.f) playerVel = playerVel - n * vn;

      if (n.y > 0.6f) grounded = true;
      hit = true;
    }
  }

  // ground
  if (enableGround) {
    float bottom = center.y - radius;
    if (bottom < groundY) {
      center.y += (groundY - bottom);
      if (playerVel.y < 0.f) playerVel.y = 0.f;
      grounded = true;
      hit = true;
    }
  }

  if (outGrounded) *outGrounded = grounded;
  return hit;
}

// ------------------------------------------------------------
// step
// ------------------------------------------------------------
void PhysicsWorldRB::substep(float h) {
  for (auto& b : m_bodies) integrate(b, h);

  std::vector<Contact> contacts;
  gatherContacts(contacts);

  for (int it = 0; it < velocityIters; ++it) {
    for (const auto& c : contacts) solveVelocity(c);
  }

  for (int it = 0; it < positionIters; ++it) {
    for (const auto& c : contacts) solvePosition(c);
  }

  for (auto& b : m_bodies) updateSleeping(b, h);
}

void PhysicsWorldRB::step(float dt) {
  if (dt <= 0.f) return;

  m_accum += dt;
  int steps = 0;

  while (m_accum >= fixedDt && steps < maxSubsteps) {
    substep(fixedDt);
    m_accum -= fixedDt;
    steps++;
  }
}

} // namespace fe
