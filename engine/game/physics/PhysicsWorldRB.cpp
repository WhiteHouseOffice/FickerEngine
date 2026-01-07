#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <cstdio>

// Defensive physics: prevent NaNs and also prevent "finite but insane" states
// that make bodies disappear or stop colliding.

namespace fe {

static inline float dot3(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float len2(const Vec3& v){ return dot3(v,v); }

static inline bool finite1(float f) { return std::isfinite(f); }
static inline bool finite3(const Vec3& v) {
  return finite1(v.x) && finite1(v.y) && finite1(v.z);
}

static inline bool finiteQuat(const Quat& q) {
  return finite1(q.w) && finite1(q.x) && finite1(q.y) && finite1(q.z);
}

static inline Vec3 clampVec3(const Vec3& v, const Vec3& mn, const Vec3& mx) {
  return Vec3(
    (v.x < mn.x) ? mn.x : (v.x > mx.x ? mx.x : v.x),
    (v.y < mn.y) ? mn.y : (v.y > mx.y ? mx.y : v.y),
    (v.z < mn.z) ? mn.z : (v.z > mx.z ? mx.z : v.z)
  );
}

static inline bool safeNormalize(Vec3& v) {
  const float l2 = len2(v);
  if (!finite1(l2) || l2 < 1e-12f) return false;
  const float invL = 1.0f / std::sqrt(l2);
  v = v * invL;
  return finite3(v);
}

// quatNormalize() can produce NaNs if input is near-zero length.
// Make it safe so physics never generates NaN orientations.
static inline Quat safeQuatNormalize(const Quat& q) {
  const float l2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (!finite1(l2) || l2 < 1e-20f) return Quat::identity();
  const float invL = 1.0f / std::sqrt(l2);
  Quat out{ q.w*invL, q.x*invL, q.y*invL, q.z*invL };
  if (!finiteQuat(out)) return Quat::identity();
  return out;
}

// ------------------------------------------------------------
// "Finite but insane" guardrails
// ------------------------------------------------------------
static constexpr float kWorldKillY   = -50.0f;   // far below world
static constexpr float kWorldMaxAbs  = 1000.0f;  // huge coordinate => went insane
static constexpr float kRespawnY     = 10.0f;    // safe height
static constexpr float kMinExtent    = 0.02f;
static constexpr float kMaxExtent    = 100.0f;

static bool extentValid(const Vec3& he) {
  if (!finite3(he)) return false;
  if (he.x < kMinExtent || he.y < kMinExtent || he.z < kMinExtent) return false;
  if (he.x > kMaxExtent || he.y > kMaxExtent || he.z > kMaxExtent) return false;
  return true;
}

static bool positionInsane(const Vec3& p) {
  if (!finite3(p)) return true;
  if (p.y < kWorldKillY) return true;
  if (std::fabs(p.x) > kWorldMaxAbs) return true;
  if (std::fabs(p.y) > kWorldMaxAbs) return true;
  if (std::fabs(p.z) > kWorldMaxAbs) return true;
  return false;
}

static void sanitizeBody(RigidBoxBody& b, const char* where) {
  bool bad = false;

  if (positionInsane(b.position)) bad = true;
  if (!finite3(b.linearVelocity) || !finite3(b.angularVelocity)) bad = true;
  if (!finiteQuat(b.orientation)) bad = true;
  if (!extentValid(b.halfExtents)) bad = true;

  if (!bad) return;

  std::printf("[PhysicsWorldRB] SANITIZE (%s) id=%u pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f) he=(%.3f %.3f %.3f) quat=(%.3f %.3f %.3f %.3f)\n",
    where, (unsigned)b.id,
    b.position.x, b.position.y, b.position.z,
    b.linearVelocity.x, b.linearVelocity.y, b.linearVelocity.z,
    b.halfExtents.x, b.halfExtents.y, b.halfExtents.z,
    b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z
  );

  // Reset to safe state (NO deletion).
  b.position = Vec3(0.0f, kRespawnY, 0.0f);
  b.linearVelocity = Vec3(0,0,0);
  b.angularVelocity = Vec3(0,0,0);
  b.orientation = Quat::identity();

  // Keep original extents if at least finite-ish; otherwise set a sane default.
  if (!extentValid(b.halfExtents)) {
    b.halfExtents = Vec3(0.5f, 0.5f, 0.5f);
  }

  b.asleep = true;
  b.sleepTimer = b.sleepTime;
}

// ------------------------------------------------------------
// Core API
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

  // ensure sane extents
  if (!extentValid(b.halfExtents)) b.halfExtents = Vec3(0.5f,0.5f,0.5f);

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

Mat3 PhysicsWorldRB::invInertiaWorld(const RigidBoxBody& b) const {
  const Quat qn = safeQuatNormalize(b.orientation);
  const Mat3 R = quatToMat3(qn);
  const Mat3 Rt = mat3Transpose(R);
  return mat3Mul(mat3Mul(R, b.invInertiaLocal), Rt);
}

void PhysicsWorldRB::applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r) {
  if (!b.isDynamic()) return;

  b.linearVelocity = b.linearVelocity + impulse * b.invMass;

  const Mat3 invIw = invInertiaWorld(b);
  const Vec3 dw = mat3Mul(invIw, cross(r, impulse));
  b.angularVelocity = b.angularVelocity + dw;

  // If something went non-finite, freeze motion; don't delete.
  if (!finite3(b.linearVelocity) || !finite3(b.angularVelocity)) {
    b.linearVelocity = Vec3(0,0,0);
    b.angularVelocity = Vec3(0,0,0);
    b.asleep = true;
    b.sleepTimer = b.sleepTime;
  }
}

void PhysicsWorldRB::integrateOrientation(RigidBoxBody& b, float h) {
  Quat q = safeQuatNormalize(b.orientation);
  const Vec3 w = b.angularVelocity;
  if (!finite3(w)) {
    b.angularVelocity = Vec3(0,0,0);
    b.asleep = true;
    b.sleepTimer = b.sleepTime;
    return;
  }

  Quat omega{0.f, w.x, w.y, w.z};
  Quat dq = quatMul(omega, q);
  q.w += 0.5f * dq.w * h;
  q.x += 0.5f * dq.x * h;
  q.y += 0.5f * dq.y * h;
  q.z += 0.5f * dq.z * h;

  b.orientation = safeQuatNormalize(q);
}

void PhysicsWorldRB::applyDamping(RigidBoxBody& b, float h) {
  if (b.linearDamping > 0.f) {
    const float k = std::max(0.f, 1.f - b.linearDamping * h);
    b.linearVelocity = b.linearVelocity * k;
  }
  if (b.angularDamping > 0.f) {
    const float k = std::max(0.f, 1.f - b.angularDamping * h);
    b.angularVelocity = b.angularVelocity * k;
  }
}

void PhysicsWorldRB::integrate(RigidBoxBody& b, float h) {
  if (!b.isDynamic() || b.asleep) return;

  if (b.useGravity) {
    b.linearVelocity = b.linearVelocity + gravity * h;
  }

  b.position = b.position + b.linearVelocity * h;
  integrateOrientation(b, h);
  applyDamping(b, h);

  // sanitize right after integration
  sanitizeBody(b, "integrate");
}

static void computeOBBAxes(const RigidBoxBody& b, Vec3& ax, Vec3& ay, Vec3& az) {
  const Quat qn = safeQuatNormalize(b.orientation);
  ax = quatRotate(qn, Vec3(1,0,0));
  ay = quatRotate(qn, Vec3(0,1,0));
  az = quatRotate(qn, Vec3(0,0,1));
}

static Vec3 obbVertex(const RigidBoxBody& b, int sx, int sy, int sz) {
  Vec3 ax,ay,az; computeOBBAxes(b, ax,ay,az);
  return b.position
    + ax * (b.halfExtents.x * (float)sx)
    + ay * (b.halfExtents.y * (float)sy)
    + az * (b.halfExtents.z * (float)sz);
}

static bool pointInOBB(const Vec3& p, const RigidBoxBody& b) {
  Vec3 ax,ay,az; computeOBBAxes(b, ax,ay,az);
  Vec3 d = p - b.position;
  const float px = dot3(d, ax);
  const float py = dot3(d, ay);
  const float pz = dot3(d, az);
  const float eps = 1e-4f;
  return std::fabs(px) <= b.halfExtents.x + eps
      && std::fabs(py) <= b.halfExtents.y + eps
      && std::fabs(pz) <= b.halfExtents.z + eps;
}

static float projectRadius(const RigidBoxBody& b, const Vec3& axis) {
  Vec3 ax,ay,az; computeOBBAxes(b, ax,ay,az);
  return std::fabs(dot3(axis, ax)) * b.halfExtents.x
       + std::fabs(dot3(axis, ay)) * b.halfExtents.y
       + std::fabs(dot3(axis, az)) * b.halfExtents.z;
}

static bool satOBBOBB(const RigidBoxBody& A, const RigidBoxBody& B, Vec3& outN, float& outPen) {
  Vec3 Ax,Ay,Az; computeOBBAxes(A, Ax,Ay,Az);
  Vec3 Bx,By,Bz; computeOBBAxes(B, Bx,By,Bz);

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

  for (int i=0;i<15;++i) {
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
      // Normal should point from B -> A.
      bestAxis = (dot3(d, a) < 0.f) ? a : (a * -1.f);
    }
  }

  if (!finite3(bestAxis) || !finite1(bestPen)) return false;
  outN = bestAxis;
  outPen = bestPen;
  return true;
}

void PhysicsWorldRB::contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out) {
  Vec3 n; float pen;
  if (!satOBBOBB(A,B,n,pen)) return;

  std::vector<Vec3> pts;
  pts.reserve(16);

  const int s[2]={-1,1};
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(A, ix,iy,iz);
    if (pointInOBB(v,B)) pts.push_back(v);
  }
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(B, ix,iy,iz);
    if (pointInOBB(v,A)) pts.push_back(v);
  }

  if (pts.empty()) {
    Vec3 p = (A.position + B.position) * 0.5f - n * (pen * 0.5f);
    pts.push_back(p);
  }

  if (pts.size() > 4) pts.resize(4);

  for (auto& p : pts) {
    Contact c;
    c.a = A.id;
    c.b = B.id;
    c.point = p;
    c.normal = n;
    c.penetration = pen;
    out.push_back(c);
  }
}

void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  float minY = 1e30f;
  Vec3 verts[8];
  int idx = 0;

  const int s[2]={-1,1};
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(A, ix,iy,iz);
    verts[idx++] = v;
    if (v.y < minY) minY = v.y;
  }

  if (minY >= groundY) return;

  const float eps = 0.01f;
  int emitted = 0;
  for (int i=0; i<8 && emitted < 4; ++i) {
    const Vec3& v = verts[i];
    if (v.y <= minY + eps) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = v;
      c.normal = Vec3(0,1,0);
      c.penetration = (groundY - v.y);
      out.push_back(c);
      emitted++;
    }
  }

  if (emitted == 0) {
    Contact c;
    c.a = A.id; c.b = 0;
    c.point = A.position;
    c.normal = Vec3(0,1,0);
    c.penetration = (groundY - minY);
    out.push_back(c);
  }
}

void PhysicsWorldRB::contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out) {
  // Special-case: stable support contact against the TOP face of the static AABB.
  const float topY = S.max.y;

  Vec3 ax, ay, az; computeOBBAxes(A, ax, ay, az);

  Vec3 verts[8];
  int idx = 0;
  const int sgn[2] = {-1, 1};
  float minY = 1e30f;
  for (int ix : sgn) for (int iy : sgn) for (int iz : sgn) {
    Vec3 v = A.position
      + ax * (A.halfExtents.x * (float)ix)
      + ay * (A.halfExtents.y * (float)iy)
      + az * (A.halfExtents.z * (float)iz);
    verts[idx++] = v;
    if (v.y < minY) minY = v.y;
  }

  const float near = 0.08f;
  if (minY <= topY + near) {
    const float epsY = 0.02f;
    int emitted = 0;

    for (int i=0; i<8 && emitted < 4; ++i) {
      const Vec3& v = verts[i];
      if (v.y > minY + epsY) continue;

      const float tol = 0.02f;
      if (v.x < S.min.x - tol || v.x > S.max.x + tol) continue;
      if (v.z < S.min.z - tol || v.z > S.max.z + tol) continue;

      if (v.y <= topY + near) {
        Contact c;
        c.a = A.id;
        c.b = 0;
        c.point = Vec3(v.x, topY, v.z);
        c.normal = Vec3(0, 1, 0);
        c.penetration = std::max(0.0f, topY - v.y);
        out.push_back(c);
        emitted++;
      }
    }

    if (emitted > 0) return;
  }

  // Fallback: vertex-inside AABB push-out (sides/bottom collisions etc.)
  const int s[2]={-1,1};
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(A, ix,iy,iz);
    Vec3 cl = clampVec3(v, S.min, S.max);
    Vec3 d = v - cl;
    float d2 = len2(d);
    if (d2 <= 1e-10f) {
      const float dxMin = v.x - S.min.x;
      const float dxMax = S.max.x - v.x;
      const float dyMin = v.y - S.min.y;
      const float dyMax = S.max.y - v.y;
      const float dzMin = v.z - S.min.z;
      const float dzMax = S.max.z - v.z;

      Vec3 n(0,1,0);
      float pen = dyMax;

      if (dyMin < dyMax) { pen = dyMin; n = Vec3(0,-1,0); }
      else               { pen = dyMax; n = Vec3(0, 1,0); }

      float px = (dxMin < dxMax) ? dxMin : dxMax;
      if (px < pen) { pen = px; n = (dxMin < dxMax) ? Vec3(-1,0,0) : Vec3(1,0,0); }

      float pz = (dzMin < dzMax) ? dzMin : dzMax;
      if (pz < pen) { pen = pz; n = (dzMin < dzMax) ? Vec3(0,0,-1) : Vec3(0,0,1); }

      Contact c;
      c.a = A.id; c.b = 0;
      c.point = v;
      c.normal = n;
      c.penetration = pen;
      out.push_back(c);
      if (out.size() >= 4) return;
    }
  }
}

void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();
  out.reserve(64);

  for (auto& b : m_bodies) {
    if (!b.isDynamic() || b.asleep) continue;
    if (enableGround) contactsBoxGround(b, out);
    for (const auto& s : m_static) contactsBoxStaticAABB(b, s, out);
  }

  for (size_t i=0;i<m_bodies.size();++i) {
    for (size_t j=i+1;j<m_bodies.size();++j) {
      const auto& A = m_bodies[i];
      const auto& B = m_bodies[j];
      if ((!A.isDynamic() && !B.isDynamic()) || (A.asleep && B.asleep)) continue;
      contactsBoxBox(A,B,out);
    }
  }
}

void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;

  const bool hasB = (c.b != 0);
  RigidBoxBody* B = hasB ? get(c.b) : nullptr;

  Vec3 n = c.normal;
  if (!safeNormalize(n)) return;

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

  const float maxJ = 200.0f;
  if (j > maxJ) j = maxJ;

  Vec3 impulse = n * j;
  // Normal is B->A, so A gets +impulse, B gets -impulse
  applyImpulse(*A, impulse, ra);
  if (B) applyImpulse(*B, impulse * -1.f, rb);

  // friction
  rv = (A->linearVelocity + cross(A->angularVelocity, ra)) - (B ? (B->linearVelocity + cross(B->angularVelocity, rb)) : Vec3(0,0,0));
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
      float maxF = friction * std::max(j, 0.5f); // small floor prevents micro-creep
      if (jt >  maxF) jt =  maxF;
      if (jt < -maxF) jt = -maxF;

      Vec3 impT = t * jt;
      applyImpulse(*A, impT, ra);
      if (B) applyImpulse(*B, impT * -1.f, rb);
    }
  }

  A->asleep = false;
  A->sleepTimer = 0.f;
  if (B) { B->asleep=false; B->sleepTimer=0.f; }
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;
  RigidBoxBody* B = (c.b != 0) ? get(c.b) : nullptr;

  const float slop = 0.01f;
  const float percent = 0.2f;
  float pen = std::max(0.f, c.penetration - slop);
  if (pen <= 0.f) return;

  Vec3 n = c.normal;
  if (!safeNormalize(n)) return;

  float wA = A->invMass;
  float wB = B ? B->invMass : 0.f;
  float wSum = wA + wB;
  if (wSum <= 0.f) return;

  Vec3 corr = n * (percent * pen / wSum);

  const float maxCorr = 0.2f;
  const float c2 = len2(corr);
  if (c2 > maxCorr * maxCorr) {
    corr = corr * (maxCorr / std::sqrt(c2));
  }

  // Normal is B->A, so A moves along +n, B along -n
  if (A->isDynamic()) A->position = A->position + corr * wA;
  if (B && B->isDynamic()) B->position = B->position - corr * wB;

  sanitizeBody(*A, "solvePosition:A");
  if (B) sanitizeBody(*B, "solvePosition:B");
}

void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h) {
  if (!b.isDynamic()) return;

  const float v2 = len2(b.linearVelocity);
  const float w2 = len2(b.angularVelocity);

  if (v2 < 1e-4f) b.linearVelocity = Vec3(0,0,0);
  if (w2 < 1e-4f) b.angularVelocity = Vec3(0,0,0);

  if (v2 < b.sleepVel*b.sleepVel && w2 < b.sleepAngVel*b.sleepAngVel) {
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

bool PhysicsWorldRB::collidePlayerSphere(Vec3& center, float radius, Vec3& playerVel, bool* outGrounded) {
  bool hit = false;
  bool grounded = false;

  for (auto& b : m_bodies) {
    if (!b.isDynamic()) continue;

    const Quat qn = safeQuatNormalize(b.orientation);
    const Mat3 R = quatToMat3(qn);
    const Mat3 Rt = mat3Transpose(R);

    Vec3 d = center - b.position;
    Vec3 local = mat3Mul(Rt, d);

    Vec3 cl = clampVec3(local,
                        Vec3(-b.halfExtents.x, -b.halfExtents.y, -b.halfExtents.z),
                        Vec3( b.halfExtents.x,  b.halfExtents.y,  b.halfExtents.z));
    Vec3 closest = mat3Mul(R, cl) + b.position;
    Vec3 delta = center - closest;
    float d2 = len2(delta);

    if (d2 < radius*radius && d2 > 1e-10f) {
      float dist = std::sqrt(d2);
      Vec3 n = delta * (1.f/dist);
      float pen = radius - dist;

      // If this is a "side" hit, don't let the resolution normal push the player down/up.
      if (std::fabs(n.y) < 0.5f) {
        n.y = 0.0f;
        float n2 = len2(n);
        if (n2 > 1e-10f) n = n * (1.0f / std::sqrt(n2));
      }

      // Push player out
      center = center + n * pen;

      // Remove player velocity into normal
      float vn = dot3(playerVel, n);
      if (vn < 0.f) playerVel = playerVel - n * vn;

      // --- Player -> box interaction (impulse at contact point for torque) ---
      // Only apply as a horizontal impulse so we don't inject vertical "rocket" energy.
      Vec3 nh(n.x, 0.0f, n.z);
      float nh2 = len2(nh);
      if (nh2 > 1e-8f) {
        nh = nh * (1.0f / std::sqrt(nh2));

        // player moving into box => positive vInto
        float vInto = -dot3(playerVel, nh);
        if (vInto > 0.0f) {
          const float playerMass = 40.0f; // tune feel
          const float pushScale  = 1.5f;  // tune feel
          float j = playerMass * pushScale * vInto;

          const float maxJ = 30.0f;
          if (j > maxJ) j = maxJ;

          Vec3 impulse = nh * j;
          Vec3 r = closest - b.position;
          applyImpulse(b, impulse, r);

          b.asleep = false;
          b.sleepTimer = 0.f;
        }
      }

      // Standing on a box: apply a small downward "weight" impulse at the contact.
      // This allows tipping when standing off-center.
      if (n.y > 0.6f) {
        grounded = true;

        const float playerMass = 40.0f;
        const float g = std::fabs(gravity.y);
        float jW = playerMass * g * fixedDt; // impulse ~ F*dt

        const float maxJW = 20.0f;
        if (jW > maxJW) jW = maxJW;

        Vec3 impulseW(0.0f, -jW, 0.0f);
        Vec3 rW = closest - b.position;
        applyImpulse(b, impulseW, rW);

        b.asleep = false;
        b.sleepTimer = 0.f;
      }

      hit = true;
      b.asleep = false;
      b.sleepTimer = 0.f;
    }
  }

  // collide vs static AABBs (player vs level geometry)
  for (const auto& s : m_static) {
    Vec3 cl = clampVec3(center, s.min, s.max);
    Vec3 d = center - cl;
    float d2 = len2(d);
    if (d2 < radius*radius) {
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

void PhysicsWorldRB::substep(float h) {
  for (auto& b : m_bodies) integrate(b, h);

  std::vector<Contact> contacts;
  gatherContacts(contacts);

  for (int it=0; it<velocityIters; ++it) {
    for (const auto& c : contacts) solveVelocity(c);
  }

  for (int it=0; it<positionIters; ++it) {
    for (const auto& c : contacts) solvePosition(c);
  }

  for (auto& b : m_bodies) {
    updateSleeping(b, h);
    sanitizeBody(b, "substep:end");
  }
}

void PhysicsWorldRB::step(float dt) {
  if (dt <= 0.f) return;
  m_accum += dt;
  int steps=0;
  while (m_accum >= fixedDt && steps < maxSubsteps) {
    substep(fixedDt);
    m_accum -= fixedDt;
    steps++;
  }
}

} // namespace fe
