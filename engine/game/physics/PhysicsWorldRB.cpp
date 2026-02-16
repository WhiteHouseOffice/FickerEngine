#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace fe {

// ============================================================
// Utilities
// ============================================================
static inline float dot3(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float len2(const Vec3& v){ return dot3(v,v); }
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b ? b : x); }

static inline bool finite1(float f) { return std::isfinite(f); }
static inline bool finite3(const Vec3& v) { return finite1(v.x) && finite1(v.y) && finite1(v.z); }

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

static inline float absf(float x){ return x < 0.f ? -x : x; }

// thresholds
static constexpr float kSupportNMin = 0.75f;
static constexpr float kUpwardVelGate = 0.5f;

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
// Integration / inertia
// ============================================================
Mat3 PhysicsWorldRB::invInertiaWorld(const RigidBoxBody& b) const {
  Quat qn = safeQuatNormalize(b.orientation);
  Mat3 R = quatToMat3(qn);
  return mat3Mul(mat3Mul(R, b.invInertiaLocal), mat3Transpose(R));
}

void PhysicsWorldRB::applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r) {
  if (!b.isDynamic()) return;

  // wake
  if (b.asleep && len2(impulse) > 1e-8f) {
    b.asleep = false;
    b.sleepTimer = 0.f;
  }

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

void PhysicsWorldRB::applyDamping(RigidBoxBody& b, float h) {
  if (!b.isDynamic() || b.asleep) return;

  const float ld = std::max(0.f, b.linearDamping);
  const float ad = std::max(0.f, b.angularDamping);

  // stable: v *= 1/(1 + d*dt)
  const float linMul = 1.f / (1.f + ld * h);
  const float angMul = 1.f / (1.f + ad * h);

  b.linearVelocity = b.linearVelocity * linMul;
  b.angularVelocity = b.angularVelocity * angMul;
}

void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h, bool supported) {
  if (!b.isDynamic()) return;

  if (!supported) {
    b.asleep = false;
    b.sleepTimer = 0.f;
    return;
  }

  const float lv2 = len2(b.linearVelocity);
  const float av2 = len2(b.angularVelocity);

  const float vThresh2 = b.sleepVel * b.sleepVel;
  const float wThresh2 = b.sleepAngVel * b.sleepAngVel;

  if (lv2 < vThresh2 && av2 < wThresh2) {
    b.sleepTimer += h;
    if (b.sleepTimer >= b.sleepTime) {
      b.asleep = true;
      b.linearVelocity = Vec3(0,0,0);
      b.angularVelocity = Vec3(0,0,0);
    }
  } else {
    b.asleep = false;
    b.sleepTimer = 0.f;
  }
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
    + ax * (b.halfExtents.x * (float)sx)
    + ay * (b.halfExtents.y * (float)sy)
    + az * (b.halfExtents.z * (float)sz);
}

static bool pointInOBB(const Vec3& p, const RigidBoxBody& b) {
  Vec3 ax, ay, az;
  computeOBBAxes(b, ax, ay, az);
  Vec3 d = p - b.position;
  float px = dot3(d, ax);
  float py = dot3(d, ay);
  float pz = dot3(d, az);
  const float eps = 1e-4f;
  return std::fabs(px) <= b.halfExtents.x + eps
      && std::fabs(py) <= b.halfExtents.y + eps
      && std::fabs(pz) <= b.halfExtents.z + eps;
}

static float projectRadius(const RigidBoxBody& b, const Vec3& axis) {
  Vec3 ax, ay, az;
  computeOBBAxes(b, ax, ay, az);
  return std::fabs(dot3(axis, ax)) * b.halfExtents.x
       + std::fabs(dot3(axis, ay)) * b.halfExtents.y
       + std::fabs(dot3(axis, az)) * b.halfExtents.z;
}

// SAT for OBB vs OBB.
// outN points from B -> A (solver convention).
static bool satOBBOBB(const RigidBoxBody& A, const RigidBoxBody& B, Vec3& outN, float& outPen) {
  Vec3 Ax, Ay, Az; computeOBBAxes(A, Ax, Ay, Az);
  Vec3 Bx, By, Bz; computeOBBAxes(B, Bx, By, Bz);

  Vec3 axes[15] = {
    Ax,Ay,Az,
    Bx,By,Bz,
    cross(Ax,Bx), cross(Ax,By), cross(Ax,Bz),
    cross(Ay,Bx), cross(Ay,By), cross(Ay,Bz),
    cross(Az,Bx), cross(Az,By), cross(Az,Bz)
  };

  Vec3 d = B.position - A.position;

  float bestPen = 1e30f;
  Vec3 bestAxis{0,1,0};

  for (int i=0; i<15; ++i) {
    Vec3 a = axes[i];
    float l2 = len2(a);
    if (!finite1(l2) || l2 < 1e-10f) continue;
    a = a * (1.f / std::sqrt(l2));

    float ra = projectRadius(A, a);
    float rb = projectRadius(B, a);
    float dist = std::fabs(dot3(d, a));
    float pen = (ra + rb) - dist;
    if (pen < 0.f) return false;

    if (pen < bestPen) {
      bestPen = pen;
      // a points roughly A->B when dot(d,a)>0; want B->A => flip
      bestAxis = (dot3(d, a) > 0.f) ? (a * -1.f) : a;
    }
  }

  outN = bestAxis;
  outPen = bestPen;
  return true;
}

// ============================================================
// Contacts
// ============================================================
void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  const float skin = std::max(0.001f, contactSkin);

  float minY = 1e30f;
  Vec3 verts[8];
  int idx = 0;

  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s) {
    Vec3 v = obbVertex(A, ix, iy, iz);
    verts[idx++] = v;
    minY = std::min(minY, v.y);
  }

  // If clearly above ground + skin, no contact.
  if (minY > groundY + skin) return;

  // Emit up to 4 bottom-most vertices, but allow those slightly above ground (skin band).
  int emitted = 0;
  const float bottomBand = minY + 0.02f;
  const float planeBand = groundY + skin;

  for (int i=0;i<8 && emitted<4;i++) {
    if (verts[i].y <= bottomBand && verts[i].y <= planeBand) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = verts[i];
      c.normal = Vec3(0,1,0); // from ground -> box (B/static -> A)
      c.penetration = std::max(0.f, groundY - verts[i].y);
      out.push_back(c);
      emitted++;
    }
  }

  // Fallback: ensure at least 1 contact in skin band
  if (emitted == 0) {
    int best = 0;
    for (int i=1;i<8;i++) if (verts[i].y < verts[best].y) best = i;

    Contact c;
    c.a = A.id;
    c.b = 0;
    c.point = verts[best];
    c.normal = Vec3(0,1,0);
    c.penetration = std::max(0.f, groundY - verts[best].y);
    out.push_back(c);
  }
}

void PhysicsWorldRB::contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out) {
  const float topY = S.max.y;
  const float skin = std::max(0.001f, contactSkin);

  Vec3 ax, ay, az;
  computeOBBAxes(A, ax, ay, az);

  Vec3 verts[8];
  float minY = 1e30f;
  int idx = 0;
  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s) {
    Vec3 v = A.position
      + ax * (A.halfExtents.x * (float)ix)
      + ay * (A.halfExtents.y * (float)iy)
      + az * (A.halfExtents.z * (float)iz);
    verts[idx++] = v;
    minY = std::min(minY, v.y);
  }

  if (minY > topY + skin) return;
  if (A.linearVelocity.y > kUpwardVelGate) return;

  const float bottomEps = 0.02f;
  const float edgeTol   = 0.10f;

  int emitted = 0;
  for (int i=0; i<8 && emitted<4; ++i) {
    const Vec3& v = verts[i];
    if (v.y > minY + bottomEps) continue;

    if (v.x < S.min.x - edgeTol || v.x > S.max.x + edgeTol) continue;
    if (v.z < S.min.z - edgeTol || v.z > S.max.z + edgeTol) continue;

    float px = clampf(v.x, S.min.x, S.max.x);
    float pz = clampf(v.z, S.min.z, S.max.z);

    if (v.y <= topY + skin) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = Vec3(px, topY, pz);
      c.normal = Vec3(0,1,0); // from platform -> box
      c.penetration = std::max(0.f, topY - v.y);
      out.push_back(c);
      emitted++;
    }
  }
}

void PhysicsWorldRB::contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out) {
  Vec3 n; float pen;
  if (!satOBBOBB(A, B, n, pen)) return;

  std::vector<Vec3> pts;
  pts.reserve(16);

  const int s[2] = {-1, 1};
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(A, ix, iy, iz);
    if (pointInOBB(v, B)) pts.push_back(v);
  }
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(B, ix, iy, iz);
    if (pointInOBB(v, A)) pts.push_back(v);
  }

  if (pts.empty()) {
    Vec3 p = (A.position + B.position) * 0.5f + n * (pen * 0.5f);
    pts.push_back(p);
  }

  // keep at most 4
  if (pts.size() > 4) pts.resize(4);

  // CRITICAL FIX:
  // Distribute penetration across manifold points to avoid position solver exploding.
  const float penEach = pen / (float)pts.size();

  for (const Vec3& p : pts) {
    Contact c;
    c.a = A.id;
    c.b = B.id;
    c.point = p;
    c.normal = n;        // B -> A (solver convention)
    c.penetration = penEach;
    out.push_back(c);
  }
}

void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();
  out.reserve(128);

  // dynamic vs ground/static
  for (auto& b : m_bodies) {
    if (!b.isDynamic() || b.asleep) continue;
    if (enableGround) contactsBoxGround(b, out);
    for (const auto& s : m_static) contactsBoxStaticAABB(b, s, out);
  }

  // dynamic vs dynamic
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    for (size_t j = i + 1; j < m_bodies.size(); ++j) {
      const auto& A = m_bodies[i];
      const auto& B = m_bodies[j];
      if ((!A.isDynamic() && !B.isDynamic()) || (A.asleep && B.asleep)) continue;
      contactsBoxBox(A, B, out);
    }
  }
}

// ============================================================
// Solver
// ============================================================
void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;

  const bool hasB = (c.b != 0);
  RigidBoxBody* B = hasB ? get(c.b) : nullptr;

  Vec3 n = c.normal; // B/static -> A
  float n2 = dot3(n, n);
  if (!finite1(n2) || n2 < 1e-12f) return;
  n = n * (1.0f / std::sqrt(n2));

  Vec3 ra = c.point - A->position;
  Vec3 va = A->linearVelocity + cross(A->angularVelocity, ra);

  Vec3 vb{0,0,0};
  Vec3 rb{0,0,0};
  float invMassB = 0.f;
  Mat3 invIBw{};

  if (B) {
    rb = c.point - B->position;
    vb = B->linearVelocity + cross(B->angularVelocity, rb);
    invMassB = B->invMass;
    invIBw = invInertiaWorld(*B);
  }

  Vec3 rv = va - vb;
  float vn = dot3(rv, n);
  if (vn > 0.f) return;

  Mat3 invIAw = invInertiaWorld(*A);

  Vec3 raXn = cross(ra, n);
  Vec3 termA = mat3Mul(invIAw, raXn);
  float kA = A->invMass + dot3(cross(termA, ra), n);

  float kB = 0.f;
  if (B) {
    Vec3 rbXn = cross(rb, n);
    Vec3 termB = mat3Mul(invIBw, rbXn);
    kB = invMassB + dot3(cross(termB, rb), n);
  }

  float denom = kA + kB;
  if (!finite1(denom) || denom <= 1e-8f) return;

  const float e = (std::fabs(vn) < 1.0f) ? 0.0f : restitution;

  float j = -(1.f + e) * vn / denom;
  if (!finite1(j)) return;

  // Slightly lower cap to reduce "touch rockets"
  const float maxJ = 50.0f;
  if (j > maxJ) j = maxJ;

  Vec3 impulse = n * j;

  applyImpulse(*A, impulse, ra);
  if (B) applyImpulse(*B, impulse * -1.f, rb);

  // Friction
  rv = (A->linearVelocity + cross(A->angularVelocity, ra))
     - (B ? (B->linearVelocity + cross(B->angularVelocity, rb)) : Vec3(0,0,0));

  Vec3 t = rv - n * dot3(rv, n);
  float t2 = len2(t);
  if (t2 > 1e-10f) {
    t = t * (1.f / std::sqrt(t2));

    Vec3 raXt = cross(ra, t);
    Vec3 termAt = mat3Mul(invIAw, raXt);
    float ktA = A->invMass + dot3(cross(termAt, ra), t);

    float ktB = 0.f;
    if (B) {
      Vec3 rbXt = cross(rb, t);
      Vec3 termBt = mat3Mul(invIBw, rbXt);
      ktB = invMassB + dot3(cross(termBt, rb), t);
    }

    float denomT = ktA + ktB;
    if (denomT > 1e-8f) {
      float jt = -dot3(rv, t) / denomT;

      // ensure friction exists even when normal impulse is tiny (skin contacts)
      float maxF = friction * std::max(j, 0.75f);
      if (jt >  maxF) jt =  maxF;
      if (jt < -maxF) jt = -maxF;

      Vec3 impT = t * jt;
      applyImpulse(*A, impT, ra);
      if (B) applyImpulse(*B, impT * -1.f, rb);
    }
  }

  A->asleep = false;
  A->sleepTimer = 0.f;
  if (B) { B->asleep = false; B->sleepTimer = 0.f; }
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;
  RigidBoxBody* B = (c.b != 0) ? get(c.b) : nullptr;

  const float slop = 0.01f;
  const float percent = 0.2f;

  float pen = c.penetration - slop;
  if (pen <= 0.f) return;

  Vec3 n = c.normal;
  float n2 = dot3(n, n);
  if (!finite1(n2) || n2 < 1e-12f) return;
  n = n * (1.0f / std::sqrt(n2));

  float wA = A->invMass;
  float wB = B ? B->invMass : 0.f;
  float wSum = wA + wB;
  if (wSum <= 0.f) return;

  Vec3 corr = n * (percent * pen / wSum);

  const float maxCorr = 0.12f; // slightly tighter
  float c2 = len2(corr);
  if (c2 > maxCorr * maxCorr) {
    corr = corr * (maxCorr / std::sqrt(c2));
  }

  if (A->isDynamic()) A->position = A->position + corr * wA;
  if (B && B->isDynamic()) B->position = B->position - corr * wB;
}

// ============================================================
// Support detection (for stabilization + sleeping)
// ============================================================
static void computeSupported(const std::vector<RigidBoxBody>& bodies,
                             const std::vector<Contact>& contacts,
                             std::vector<uint8_t>& supportedOut) {
  supportedOut.assign(bodies.size(), 0);

  auto idxById = [&](uint32_t id)->int {
    for (int i=0;i<(int)bodies.size();++i) if (bodies[(size_t)i].id == id) return i;
    return -1;
  };

  for (const auto& c : contacts) {
    if (c.normal.y < kSupportNMin) continue;
    int ia = idxById(c.a);
    if (ia < 0) continue;
    supportedOut[(size_t)ia] = 1;
  }
}

// Keep your existing stabilization pass, but don’t change behavior much.
// (This is still a “nice to have”; the real fixes are skin + damping + sleeping + pen distribution.)
static void stabilizeSupportedBodies(std::vector<RigidBoxBody>& bodies,
                                    const std::vector<uint8_t>& supported) {
  for (size_t i=0;i<bodies.size();++i) {
    if (!supported[i]) continue;
    auto& b = bodies[i];
    if (!b.isDynamic() || b.asleep) continue;

    // kill tiny lateral drift
    Vec3 lateral(b.linearVelocity.x, 0.f, b.linearVelocity.z);
    if (len2(lateral) < 0.25f*0.25f) {
      b.linearVelocity.x *= 0.45f;
      b.linearVelocity.z *= 0.45f;
    }

    // kill tiny angular jitter
    if (len2(b.angularVelocity) < 1.25f*1.25f) {
      b.angularVelocity = b.angularVelocity * 0.45f;
    }
  }
}

// ============================================================
// Player collision (unchanged from your version)
// ============================================================
bool PhysicsWorldRB::collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded) {
  bool hit = false;
  bool grounded = false;

  Vec3 playerVelIn = playerVel;

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
    Vec3 nRaw = delta * (1.f / dist); // box -> player
    Vec3 nRes = nRaw;

    if (std::fabs(nRes.y) < 0.5f) {
      nRes.y = 0.f;
      safeNormalize(nRes);
    }

    center = center + nRes * (radius - dist);

    float vn = dot3(playerVel, nRes);
    if (vn < 0.f) playerVel -= nRes * vn;

    Vec3 nh(nRaw.x, 0.0f, nRaw.z);
    if (safeNormalize(nh)) {
      if (std::fabs(nRaw.y) < 0.65f) {
        float vInto = -dot3(playerVelIn, nh);
        if (vInto > 0.f) {
          float j = 70.0f * vInto;
          if (j > 22.0f) j = 22.0f;
          applyImpulse(b, nh * (-j), Vec3(0,0,0));
          b.asleep = false;
          b.sleepTimer = 0.f;
        }
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
  for (auto& b : m_bodies) {
    integrate(b, h);
    applyDamping(b, h);
  }

  std::vector<Contact> contacts;
  gatherContacts(contacts);

  for (int it=0; it<velocityIters; ++it)
    for (const auto& c : contacts) solveVelocity(c);

  for (int it=0; it<positionIters; ++it)
    for (const auto& c : contacts) solvePosition(c);

  std::vector<uint8_t> supported;
  computeSupported(m_bodies, contacts, supported);

  stabilizeSupportedBodies(m_bodies, supported);

  for (size_t i=0;i<m_bodies.size();++i) {
    updateSleeping(m_bodies[i], h, supported[i] != 0);
  }
}

void PhysicsWorldRB::step(float dt) {
  m_accum += dt;
  while (m_accum >= fixedDt) {
    substep(fixedDt);
    m_accum -= fixedDt;
  }
}

} // namespace fe
