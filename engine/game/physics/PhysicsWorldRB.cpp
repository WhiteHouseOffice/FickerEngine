#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <cstdio>

namespace fe {

// ============================================================
// Utilities
// ============================================================
static inline float dot3(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float len2(const Vec3& v){ return dot3(v,v); }

static inline bool finite1(float f) { return std::isfinite(f); }
static inline bool finite3(const Vec3& v) {
  return finite1(v.x) && finite1(v.y) && finite1(v.z);
}

static inline Vec3 clampVec3(const Vec3& v, const Vec3& mn, const Vec3& mx) {
  return Vec3(
    (v.x < mn.x) ? mn.x : (v.x > mx.x ? mx.x : v.x),
    (v.y < mn.y) ? mn.y : (v.y > mx.y ? mx.y : v.y),
    (v.z < mn.z) ? mn.z : (v.z > mx.z ? mx.z : v.z)
  );
}

static inline bool safeNormalize(Vec3& v) {
  float l2 = len2(v);
  if (!finite1(l2) || l2 < 1e-12f) return false;
  v = v * (1.0f / std::sqrt(l2));
  return finite3(v);
}

static inline Quat safeQuatNormalize(const Quat& q) {
  float l2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (!finite1(l2) || l2 < 1e-20f) return Quat::identity();
  float inv = 1.0f / std::sqrt(l2);
  return Quat{ q.w*inv, q.x*inv, q.y*inv, q.z*inv };

}

// ============================================================
// Creation / access
// ============================================================
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

// ============================================================
// Integration
// ============================================================
Mat3 PhysicsWorldRB::invInertiaWorld(const RigidBoxBody& b) const {
  Quat qn = safeQuatNormalize(b.orientation);
  Mat3 R = quatToMat3(qn);
  return mat3Mul(mat3Mul(R, b.invInertiaLocal), mat3Transpose(R));
}

void PhysicsWorldRB::applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r) {
  if (!b.isDynamic()) return;

  b.linearVelocity = b.linearVelocity + impulse * b.invMass;
  Vec3 dw = mat3Mul(invInertiaWorld(b), cross(r, impulse));
  b.angularVelocity = b.angularVelocity + dw;
}

void PhysicsWorldRB::integrateOrientation(RigidBoxBody& b, float h) {
  Quat q = safeQuatNormalize(b.orientation);
  Vec3 w = b.angularVelocity;
  Quat omega{ 0.f, w.x, w.y, w.z };
  Quat dq = quatMul(omega, q);

  q.w += 0.5f * dq.w * h;
  q.x += 0.5f * dq.x * h;
  q.y += 0.5f * dq.y * h;
  q.z += 0.5f * dq.z * h;

  b.orientation = safeQuatNormalize(q);
}

void PhysicsWorldRB::integrate(RigidBoxBody& b, float h) {
  if (!b.isDynamic() || b.asleep) return;

  if (b.useGravity)
    b.linearVelocity = b.linearVelocity + gravity * h;

  b.position = b.position + b.linearVelocity * h;
  integrateOrientation(b, h);
}

// ============================================================
// Collision helpers
// ============================================================
static void computeOBBAxes(const RigidBoxBody& b, Vec3& ax, Vec3& ay, Vec3& az) {
  Quat qn = safeQuatNormalize(b.orientation);
  ax = quatRotate(qn, Vec3(1,0,0));
  ay = quatRotate(qn, Vec3(0,1,0));
  az = quatRotate(qn, Vec3(0,0,1));
}

static Vec3 obbVertex(const RigidBoxBody& b, int sx, int sy, int sz) {
  Vec3 ax, ay, az;
  computeOBBAxes(b, ax, ay, az);
  return b.position
    + ax * (b.halfExtents.x * sx)
    + ay * (b.halfExtents.y * sy)
    + az * (b.halfExtents.z * sz);
}

// ============================================================
// Contacts
// ============================================================
void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  float minY = 1e30f;
  Vec3 verts[8];
  int idx = 0;

  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s) {
    Vec3 v = obbVertex(A, ix, iy, iz);
    verts[idx++] = v;
    minY = std::min(minY, v.y);
  }

  if (minY >= groundY) return;

  int emitted = 0;
  for (int i=0;i<8 && emitted<4;i++) {
    if (verts[i].y <= minY + 0.01f) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = verts[i];
      c.normal = Vec3(0,1,0);
      c.penetration = groundY - verts[i].y;
      out.push_back(c);
      emitted++;
    }
  }
}

void PhysicsWorldRB::contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out) {
  // Stable top-face support manifold
  float topY = S.max.y;

  Vec3 ax, ay, az;
  computeOBBAxes(A, ax, ay, az);

  Vec3 verts[8];
  float minY = 1e30f;
  int idx = 0;
  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s) {
    Vec3 v = A.position
      + ax * (A.halfExtents.x * ix)
      + ay * (A.halfExtents.y * iy)
      + az * (A.halfExtents.z * iz);
    verts[idx++] = v;
    minY = std::min(minY, v.y);
  }

  if (minY > topY + 0.08f) return;

  int emitted = 0;
  for (int i=0;i<8 && emitted<4;i++) {
    const Vec3& v = verts[i];
    if (v.y > minY + 0.02f) continue;
    if (v.x < S.min.x || v.x > S.max.x) continue;
    if (v.z < S.min.z || v.z > S.max.z) continue;

    Contact c;
    c.a = A.id;
    c.b = 0;
    c.point = Vec3(v.x, topY, v.z);
    c.normal = Vec3(0,1,0);
    c.penetration = std::max(0.f, topY - v.y);
    out.push_back(c);
    emitted++;
  }
}

void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();

  for (auto& b : m_bodies) {
    if (!b.isDynamic() || b.asleep) continue;
    if (enableGround) contactsBoxGround(b, out);
    for (const auto& s : m_static) contactsBoxStaticAABB(b, s, out);
  }
}

// ============================================================
// Solver
// ============================================================
void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;

  Vec3 n = c.normal;
  if (!safeNormalize(n)) return;

  Vec3 ra = c.point - A->position;
  Vec3 va = A->linearVelocity + cross(A->angularVelocity, ra);
  float vn = dot3(va, n);
  if (vn > 0.f) return;

  float denom = A->invMass;
  if (denom <= 0.f) return;

  float j = -(1.f + restitution) * vn / denom;
  j = std::min(j, 200.f);

  applyImpulse(*A, n * j, ra);
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;

  float pen = std::max(0.f, c.penetration - 0.01f);
  if (pen <= 0.f) return;

  Vec3 n = c.normal;
  if (!safeNormalize(n)) return;

  A->position = A->position + n * (pen * 0.2f);
}

// ============================================================
// Player collision (FIXED)
// ============================================================
bool PhysicsWorldRB::collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded) {
  bool hit = false;
  bool grounded = false;

  Vec3 playerVelIn = playerVel; // IMPORTANT

  for (auto& b : m_bodies) {
    if (!b.isDynamic()) continue;

    Quat qn = safeQuatNormalize(b.orientation);
    Mat3 R = quatToMat3(qn);
    Mat3 Rt = mat3Transpose(R);

    Vec3 local = mat3Mul(Rt, center - b.position);
    Vec3 cl = clampVec3(local,
      Vec3(-b.halfExtents.x, -b.halfExtents.y, -b.halfExtents.z),
      Vec3( b.halfExtents.x,  b.halfExtents.y,  b.halfExtents.z));

    Vec3 closest = mat3Mul(R, cl) + b.position;
    Vec3 delta = center - closest;
    float d2 = len2(delta);
    if (d2 >= radius*radius || d2 < 1e-10f) continue;

    float dist = std::sqrt(d2);
    Vec3 nRaw = delta * (1.f / dist);
    Vec3 nRes = nRaw;

    if (std::fabs(nRes.y) < 0.5f) {
      nRes.y = 0.f;
      safeNormalize(nRes);
    }

    center = center + nRes * (radius - dist);

    float vn = dot3(playerVel, nRes);
    if (vn < 0.f) playerVel -= nRes * vn;

    // push box using INCOMING velocity
    Vec3 nh(nRaw.x, 0, nRaw.z);
    if (safeNormalize(nh)) {
      float vInto = -dot3(playerVelIn, nh);
      if (vInto > 0.f) {
        float j = std::min(60.f * vInto, 18.f);
        applyImpulse(b, nh * j, closest - b.position);
        b.asleep = false;
      }
    }

    if (nRaw.y > 0.6f) grounded = true;
    hit = true;
  }

  if (outGrounded) *outGrounded = grounded;
  return hit;
}

// ============================================================
// Stepping
// ============================================================
void PhysicsWorldRB::substep(float h) {
  for (auto& b : m_bodies) integrate(b, h);

  std::vector<Contact> contacts;
  gatherContacts(contacts);

  for (int it=0; it<velocityIters; ++it)
    for (const auto& c : contacts) solveVelocity(c);

  for (int it=0; it<positionIters; ++it)
    for (const auto& c : contacts) solvePosition(c);
}

void PhysicsWorldRB::step(float dt) {
  m_accum += dt;
  while (m_accum >= fixedDt) {
    substep(fixedDt);
    m_accum -= fixedDt;
  }
}

} // namespace fe
