#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>

namespace fe {

// ------------------------------------------------------------
// helpers
// ------------------------------------------------------------
static inline float dot3(const Vec3& a, const Vec3& b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline float len2(const Vec3& v){
  return dot3(v,v);
}

static inline Vec3 clampVec3(const Vec3& v, const Vec3& mn, const Vec3& mx){
  return Vec3(
    (v.x < mn.x) ? mn.x : (v.x > mx.x ? mx.x : v.x),
    (v.y < mn.y) ? mn.y : (v.y > mx.y ? mx.y : v.y),
    (v.z < mn.z) ? mn.z : (v.z > mx.z ? mx.z : v.z)
  );
}

// ------------------------------------------------------------
// creation / access
// ------------------------------------------------------------
uint32_t PhysicsWorldRB::createBox(const Vec3& pos, const Vec3& halfExtents, float mass){
  RigidBoxBody b;
  b.id = m_nextId++;
  b.position = pos;
  b.orientation = Quat::identity();
  b.halfExtents = halfExtents;

  if (mass > 0.f){
    b.invMass = 1.f / mass;
    b.invInertiaLocal = boxInertiaInvLocal(halfExtents, mass);
  } else {
    b.invMass = 0.f;
    b.invInertiaLocal = Mat3{};
  }

  m_bodies.push_back(b);
  return b.id;
}

RigidBoxBody* PhysicsWorldRB::get(uint32_t id){
  for (auto& b : m_bodies)
    if (b.id == id) return &b;
  return nullptr;
}

void PhysicsWorldRB::clearDynamics(){
  m_bodies.clear();
  m_nextId = 1;
  m_accum = 0.f;
}

// ------------------------------------------------------------
// integration
// ------------------------------------------------------------
void PhysicsWorldRB::integrate(RigidBoxBody& b, float h){
  if (!b.isDynamic() || b.asleep) return;

  if (b.useGravity)
    b.linearVelocity = b.linearVelocity + gravity * h;

  b.position = b.position + b.linearVelocity * h;

  // NOTE: orientation integration intentionally disabled
  // until angular impulses are re-enabled safely.
}

// ------------------------------------------------------------
// contacts
// ------------------------------------------------------------
void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out){
  float minY = 1e30f;
  Vec3 minP = A.position;

  const int s[2] = { -1, 1 };
  for (int ix : s)
    for (int iy : s)
      for (int iz : s){
        Vec3 v = A.position + Vec3(
          A.halfExtents.x * ix,
          A.halfExtents.y * iy,
          A.halfExtents.z * iz
        );
        if (v.y < minY){
          minY = v.y;
          minP = v;
        }
      }

  if (minY < groundY){
    Contact c;
    c.a = A.id;
    c.b = 0;
    c.point = minP;
    c.normal = Vec3(0,1,0);
    c.penetration = groundY - minY;
    out.push_back(c);
  }
}

void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out){
  out.clear();
  out.reserve(32);

  for (auto& b : m_bodies){
    if (!b.isDynamic() || b.asleep) continue;
    if (enableGround)
      contactsBoxGround(b, out);
  }
}

// ------------------------------------------------------------
// solver (LINEAR ONLY — SAFE)
// ------------------------------------------------------------
void PhysicsWorldRB::solveVelocity(const Contact& c){
  RigidBoxBody* A = get(c.a);
  if (!A) return;

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

  A->linearVelocity = A->linearVelocity + n * (j * A->invMass);

  A->asleep = false;
  A->sleepTimer = 0.f;
}

void PhysicsWorldRB::solvePosition(const Contact& c){
  RigidBoxBody* A = get(c.a);
  if (!A || !A->isDynamic()) return;

  float pen = std::max(0.f, c.penetration - 0.002f);
  if (pen <= 0.f) return;

  A->position = A->position + c.normal * pen;
}

// ------------------------------------------------------------
// sleeping
// ------------------------------------------------------------
void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h){
  if (!b.isDynamic()) return;

  if (len2(b.linearVelocity) < b.sleepVel * b.sleepVel){
    b.sleepTimer += h;
    if (b.sleepTimer >= b.sleepTime){
      b.asleep = true;
      b.linearVelocity = Vec3(0,0,0);
    }
  } else {
    b.sleepTimer = 0.f;
    b.asleep = false;
  }
}

// ------------------------------------------------------------
// step
// ------------------------------------------------------------
void PhysicsWorldRB::substep(float h){
  for (auto& b : m_bodies)
    integrate(b, h);

  std::vector<Contact> contacts;
  gatherContacts(contacts);

  for (int it = 0; it < velocityIters; ++it)
    for (const auto& c : contacts)
      solveVelocity(c);

  for (int it = 0; it < positionIters; ++it)
    for (const auto& c : contacts)
      solvePosition(c);

  for (auto& b : m_bodies)
    updateSleeping(b, h);
}

void PhysicsWorldRB::step(float dt){
  if (dt <= 0.f) return;

  m_accum += dt;
  int steps = 0;

  while (m_accum >= fixedDt && steps < maxSubsteps){
    substep(fixedDt);
    m_accum -= fixedDt;
    steps++;
  }
}

} // namespace fe
