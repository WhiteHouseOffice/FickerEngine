#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>

namespace fe {

// ------------------------------------------------------------
// small math helpers
// ------------------------------------------------------------
static inline float dot3(const Vec3& a, const Vec3& b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline float len2(const Vec3& v){
  return dot3(v,v);
}
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
  b.orientation = Quat::identity();
  b.halfExtents = halfExtents;

  if (mass > 0.f) {
    b.invMass = 1.f / mass;
    b.invInertiaLocal = boxInertiaInvLocal(halfExtents, mass);
  } else {
    b.invMass = 0.f;
    b.invInertiaLocal = Mat3{};
  }

  m_bodies.push_back(b);
  return b.id;
}

RigidBoxBody* PhysicsWorldRB::get(uint32_t id) {
  for (auto& b : m_bodies)
    if (b.id == id) return &b;
  return nullptr;
}

void PhysicsWorldRB::clearDynamics() {
  m_bodies.clear();
  m_nextId = 1;
  m_accum = 0.f;
}

// ------------------------------------------------------------
// integration
// ------------------------------------------------------------
void PhysicsWorldRB::integrate(RigidBoxBody& b, float h) {
  if (!b.isDynamic() || b.asleep) return;

  if (b.useGravity)
    b.linearVelocity = b.linearVelocity + gravity * h;

  b.position = b.position + b.linearVelocity * h;

  // angular motion is allowed but safe
  if (len2(b.angularVelocity) > 0.f) {
    Quat q = quatNormalize(b.orientation);
    Vec3 w = b.angularVelocity;
    Quat dq{0.f, w.x, w.y, w.z};
    dq = quatMul(dq, q);

    q.w += 0.5f * dq.w * h;
    q.x += 0.5f * dq.x * h;
    q.y += 0.5f * dq.y * h;
    q.z += 0.5f * dq.z * h;
    b.orientation = quatNormalize(q);
  }

  // damping
  b.linearVelocity  = b.linearVelocity  * std::max(0.f, 1.f - b.linearDamping  * h);
  b.angularVelocity = b.angularVelocity * std::max(0.f, 1.f - b.angularDamping * h);
}

// ------------------------------------------------------------
// contacts
// ------------------------------------------------------------
void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  float bottom = A.position.y - A.halfExtents.y;
  if (bottom < groundY) {
    Contact c;
    c.a = A.id;
    c.b = 0;
    c.point = Vec3(A.position.x, groundY, A.position.z);
    c.normal = Vec3(0,1,0);
    c.penetration = groundY - bottom;
    out.push_back(c);
  }
}

void PhysicsWorldRB::contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out) {
  Vec3 Amn(A.position.x - A.halfExtents.x,
           A.position.y - A.halfExtents.y,
           A.position.z - A.halfExtents.z);
  Vec3 Amx(A.position.x + A.halfExtents.x,
           A.position.y + A.halfExtents.y,
           A.position.z + A.halfExtents.z);

  if (Amx.x < S.min.x || Amn.x > S.max.x) return;
  if (Amx.y < S.min.y || Amn.y > S.max.y) return;
  if (Amx.z < S.min.z || Amn.z > S.max.z) return;

  float px = std::min(S.max.x - Amn.x, Amx.x - S.min.x);
  float py = std::min(S.max.y - Amn.y, Amx.y - S.min.y);
  float pz = std::min(S.max.z - Amn.z, Amx.z - S.min.z);

  Vec3 n(0,1,0);
  float pen = py;

  if (px < pen) {
    pen = px;
    n = (A.position.x < (S.min.x+S.max.x)*0.5f) ? Vec3(-1,0,0) : Vec3(1,0,0);
  }
  if (pz < pen) {
    pen = pz;
    n = (A.position.z < (S.min.z+S.max.z)*0.5f) ? Vec3(0,0,-1) : Vec3(0,0,1);
  }

  Contact c;
  c.a = A.id;
  c.b = 0;
  c.point = A.position;
  c.normal = n;
  c.penetration = pen;
  out.push_back(c);
}

void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();

  for (auto& b : m_bodies) {
    if (!b.isDynamic() || b.asleep) continue;

    if (enableGround)
      contactsBoxGround(b, out);

    for (const auto& s : m_static)
      contactsBoxStaticAABB(b, s, out);
  }
}

// ------------------------------------------------------------
// solvers
// ------------------------------------------------------------
void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A || !A->isDynamic()) return;

  Vec3 n = c.normal;
  float vn = dot3(A->linearVelocity, n);
  if (vn < 0.f)
    A->linearVelocity = A->linearVelocity - n * vn;

  // friction
  Vec3 v = A->linearVelocity;
  Vec3 t = v - n * dot3(v,n);
  float t2 = len2(t);
  if (t2 > 1e-8f) {
    t = t * (1.f / std::sqrt(t2));
    A->linearVelocity = A->linearVelocity - t * (friction * vn);
  }
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A || !A->isDynamic()) return;

  const float slop = 0.001f;
  const float percent = 0.8f;
  float pen = std::max(0.f, c.penetration - slop);
  if (pen <= 0.f) return;

  A->position = A->position + c.normal * (pen * percent);
}

// ------------------------------------------------------------
// sleeping
// ------------------------------------------------------------
void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h) {
  if (!b.isDynamic()) return;

  if (len2(b.linearVelocity) < 1e-4f &&
      len2(b.angularVelocity) < 1e-4f) {
    b.sleepTimer += h;
    if (b.sleepTimer > b.sleepTime) {
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
// player collision
// ------------------------------------------------------------
bool PhysicsWorldRB::collidePlayerSphere(Vec3& center, float radius, Vec3& vel, bool* groundedOut) {
  bool grounded = false;

  for (auto& b : m_bodies) {
    if (!b.isDynamic()) continue;

    Vec3 d = center - b.position;
    Vec3 cl = clampVec3(d,
      Vec3(-b.halfExtents.x, -b.halfExtents.y, -b.halfExtents.z),
      Vec3( b.halfExtents.x,  b.halfExtents.y,  b.halfExtents.z));

    Vec3 closest = b.position + cl;
    Vec3 delta = center - closest;
    float d2 = len2(delta);

    if (d2 < radius*radius && d2 > 1e-8f) {
      float dist = std::sqrt(d2);
      Vec3 n = delta * (1.f / dist);
      float pen = radius - dist;

      center = center + n * pen;

      float vn = dot3(vel, n);
      if (vn < 0.f) vel = vel - n * vn;

      // push box
      Vec3 push = Vec3(vel.x, 0.f, vel.z);
      if (len2(push) > 1e-6f) {
        push = push * (1.f / std::sqrt(len2(push)));
        b.linearVelocity = b.linearVelocity + push * (6.f * b.invMass);
        b.asleep = false;
      }

      if (n.y > 0.6f) grounded = true;
    }
  }

  if (enableGround) {
    float bottom = center.y - radius;
    if (bottom < groundY) {
      center.y += groundY - bottom;
      if (vel.y < 0.f) vel.y = 0.f;
      grounded = true;
    }
  }

  if (groundedOut) *groundedOut = grounded;
  return grounded;
}

// ------------------------------------------------------------
// stepping
// ------------------------------------------------------------
void PhysicsWorldRB::substep(float h) {
  for (auto& b : m_bodies)
    integrate(b, h);

  std::vector<Contact> contacts;
  gatherContacts(contacts);

  for (int i=0;i<velocityIters;i++)
    for (const auto& c : contacts)
      solveVelocity(c);

  for (int i=0;i<positionIters;i++)
    for (const auto& c : contacts)
      solvePosition(c);

  for (auto& b : m_bodies)
    updateSleeping(b, h);
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
