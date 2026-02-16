#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>

namespace fe {

// ------------------------------------------------------------
// small helpers
// ------------------------------------------------------------
static inline float dot3(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3  cross3(const Vec3& a, const Vec3& b){ return cross(a,b); }
static inline float len2(const Vec3& v){ return dot3(v,v); }
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b ? b : x); }

static inline bool safeNormalize(Vec3& v) {
  float l2 = len2(v);
  if (!std::isfinite(l2) || l2 < 1e-12f) return false;
  v = v * (1.0f / std::sqrt(l2));
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static inline Quat safeQuatNormalize(const Quat& q) {
  float l2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (!std::isfinite(l2) || l2 < 1e-20f) return Quat::identity();
  float inv = 1.0f / std::sqrt(l2);
  return Quat{ q.w*inv, q.x*inv, q.y*inv, q.z*inv };
}

// ------------------------------------------------------------
// Closest point on triangle (Ericson)
// ------------------------------------------------------------
static Vec3 closestPointOnTri(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
  Vec3 ab = b - a;
  Vec3 ac = c - a;
  Vec3 ap = p - a;

  float d1 = dot3(ab, ap);
  float d2 = dot3(ac, ap);
  if (d1 <= 0.f && d2 <= 0.f) return a;

  Vec3 bp = p - b;
  float d3 = dot3(ab, bp);
  float d4 = dot3(ac, bp);
  if (d3 >= 0.f && d4 <= d3) return b;

  float vc = d1*d4 - d3*d2;
  if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
    float v = d1 / (d1 - d3);
    return a + ab * v;
  }

  Vec3 cp = p - c;
  float d5 = dot3(ab, cp);
  float d6 = dot3(ac, cp);
  if (d6 >= 0.f && d5 <= d6) return c;

  float vb = d5*d2 - d1*d6;
  if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
    float w = d2 / (d2 - d6);
    return a + ac * w;
  }

  float va = d3*d6 - d5*d4;
  if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
    float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + (c - b) * w;
  }

  float denom = 1.f / (va + vb + vc);
  float v = vb * denom;
  float w = vc * denom;
  return a + ab * v + ac * w;
}

// ------------------------------------------------------------
// OBB helpers
// ------------------------------------------------------------
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

static float projectRadius(const RigidBoxBody& b, const Vec3& axis) {
  Vec3 ax, ay, az;
  computeOBBAxes(b, ax, ay, az);
  return std::fabs(dot3(axis, ax)) * b.halfExtents.x
       + std::fabs(dot3(axis, ay)) * b.halfExtents.y
       + std::fabs(dot3(axis, az)) * b.halfExtents.z;
}

static bool satOBBOBB(const RigidBoxBody& A, const RigidBoxBody& B, Vec3& outN, float& outPen) {
  Vec3 Ax, Ay, Az; computeOBBAxes(A, Ax, Ay, Az);
  Vec3 Bx, By, Bz; computeOBBAxes(B, Bx, By, Bz);

  Vec3 axes[15] = {
    Ax,Ay,Az,
    Bx,By,Bz,
    cross3(Ax,Bx), cross3(Ax,By), cross3(Ax,Bz),
    cross3(Ay,Bx), cross3(Ay,By), cross3(Ay,Bz),
    cross3(Az,Bx), cross3(Az,By), cross3(Az,Bz)
  };

  Vec3 d = B.position - A.position;

  float bestPen = 1e30f;
  Vec3 bestAxis{0,1,0};

  for (int i=0; i<15; ++i) {
    Vec3 a = axes[i];
    float l2 = len2(a);
    if (!std::isfinite(l2) || l2 < 1e-10f) continue;
    a = a * (1.f / std::sqrt(l2));

    float ra = projectRadius(A, a);
    float rb = projectRadius(B, a);
    float dist = std::fabs(dot3(d, a));
    float pen = (ra + rb) - dist;
    if (pen < 0.f) return false;

    if (pen < bestPen) {
      bestPen = pen;
      // want normal from B -> A
      bestAxis = (dot3(d, a) > 0.f) ? (a * -1.f) : a;
    }
  }

  outN = bestAxis;
  outPen = bestPen;
  return true;
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

// ------------------------------------------------------------
// World inertia
// ------------------------------------------------------------
Mat3 PhysicsWorldRB::invInertiaWorld(const RigidBoxBody& b) const {
  Quat qn = safeQuatNormalize(b.orientation);
  Mat3 R = quatToMat3(qn);
  return mat3Mul(mat3Mul(R, b.invInertiaLocal), mat3Transpose(R));
}

void PhysicsWorldRB::applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r) {
  if (!b.isDynamic()) return;

  b.linearVelocity = b.linearVelocity + impulse * b.invMass;
  Vec3 dw = mat3Mul(invInertiaWorld(b), cross3(r, impulse));
  b.angularVelocity = b.angularVelocity + dw;

  b.asleep = false;
  b.sleepTimer = 0.f;
}

// ------------------------------------------------------------
// Create / access
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
// Terrain callbacks
// ------------------------------------------------------------
float PhysicsWorldRB::terrainHeightAt(float x, float z) const {
  if (!m_terrainHeightFn) return -1e30f;
  return m_terrainHeightFn(m_terrainUser, x, z);
}

Vec3 PhysicsWorldRB::terrainNormalAt(float x, float z) const {
  if (m_terrainNormalFn) {
    Vec3 n = m_terrainNormalFn(m_terrainUser, x, z);
    if (safeNormalize(n)) return n;
  }
  return Vec3(0,1,0);
}

// ------------------------------------------------------------
// Integrate
// ------------------------------------------------------------
void PhysicsWorldRB::integrateOrientation(RigidBoxBody& b, float h) {
  Quat q = safeQuatNormalize(b.orientation);
  Vec3 w = b.angularVelocity;
  Quat omega{ 0.f, w.x, w.y, w.z };
  Quat dq = quatMul(omega, q);

  q.w += 0.5f * dq.w * h;
  q.x += 0.5f * dq.x * h;
  q.y += 0.5f * dq.y * h;
  q.z += 0.5f * dq.z * h;

  b.orientation = quatNormalize(q);
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

  const float linMul = 1.f / (1.f + ld * h);
  const float angMul = 1.f / (1.f + ad * h);

  b.linearVelocity  = b.linearVelocity  * linMul;
  b.angularVelocity = b.angularVelocity * angMul;
}

// ------------------------------------------------------------
// Contacts: ground + terrain
// ------------------------------------------------------------
void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  const float skin = std::max(0.001f, contactSkin);

  Vec3 verts[8];
  int idx = 0;
  float minY = 1e30f;

  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s) {
    Vec3 v = obbVertex(A, ix, iy, iz);
    verts[idx++] = v;
    minY = std::min(minY, v.y);
  }

  if (minY > groundY + skin) return;

  int emitted = 0;
  const float nearMin = minY + 0.02f;
  for (int i=0;i<8 && emitted<4;i++) {
    if (verts[i].y <= nearMin && verts[i].y <= groundY + skin) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = Vec3(verts[i].x, groundY, verts[i].z);
      c.normal = Vec3(0,1,0);
      c.penetration = std::max(0.f, groundY - verts[i].y);
      out.push_back(c);
      emitted++;
    }
  }
}

void PhysicsWorldRB::contactsBoxTerrain(const RigidBoxBody& A, std::vector<Contact>& out) {
  if (!enableTerrain || !m_terrainHeightFn) return;

  const float skin = std::max(0.001f, contactSkin);

  Vec3 verts[8];
  int idx = 0;
  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s)
    verts[idx++] = obbVertex(A, ix, iy, iz);

  int emitted = 0;
  for (int i=0;i<8 && emitted<4;i++) {
    const Vec3& v = verts[i];
    const float h = terrainHeightAt(v.x, v.z);
    const float depth = h - v.y;
    if (depth >= -skin) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = Vec3(v.x, h, v.z);
      c.normal = terrainNormalAt(v.x, v.z);
      c.penetration = std::max(0.f, depth);
      out.push_back(c);
      emitted++;
    }
  }
}

// ------------------------------------------------------------
// Contacts: box vs static triangle meshes (CW-wound meshes)
// ------------------------------------------------------------
void PhysicsWorldRB::contactsBoxStaticMeshes(const RigidBoxBody& A, std::vector<Contact>& out) {
  if (m_staticMeshes.empty()) return;

  const float skin = std::max(0.001f, contactSkin);

  Vec3 boxVerts[8];
  int idx = 0;
  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s)
    boxVerts[idx++] = obbVertex(A, ix, iy, iz);

  struct Cand { Contact c; };
  std::vector<Cand> cands;
  cands.reserve(16);

  for (int vi = 0; vi < 8; ++vi) {
    const Vec3& p = boxVerts[vi];

    bool  hasBest = false;
    float bestPen = 0.f;
    Contact bestC;

    for (const auto& mesh : m_staticMeshes) {
      if (mesh.indices.size() < 3 || mesh.verts.empty()) continue;

      for (size_t ti = 0; ti + 2 < mesh.indices.size(); ti += 3) {
        const Vec3& a = mesh.verts[mesh.indices[ti+0]];
        const Vec3& b = mesh.verts[mesh.indices[ti+1]];
        const Vec3& c = mesh.verts[mesh.indices[ti+2]];

        // Meshes are CW-wound (rendering), flip normal for outward.
        Vec3 n = cross3(b - a, c - a) * -1.f;
        if (!safeNormalize(n)) continue;

        Vec3 cp = closestPointOnTri(p, a, b, c);
        float dist = dot3(p - cp, n);

        if (dist > skin) continue;

        float pen = skin - dist;
        const float slop = 0.0025f;
        if (pen <= slop) continue;
        pen -= slop;

        if (!hasBest || pen > bestPen) {
          hasBest = true;
          bestPen = pen;
          bestC.a = A.id;
          bestC.b = 0;
          bestC.point = cp;
          bestC.normal = n;
          bestC.penetration = pen;
        }
      }
    }

    if (hasBest) cands.push_back({bestC});
  }

  if (cands.empty()) return;

  std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y){
    return x.c.penetration > y.c.penetration;
  });

  const int keep = std::min<int>(8, (int)cands.size());
  for (int i=0;i<keep;++i)
    out.push_back(cands[(size_t)i].c);
}

// ------------------------------------------------------------
// Contacts: box vs box
// ------------------------------------------------------------
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

  if (pts.size() > 4) pts.resize(4);
  const float penEach = pen / (float)pts.size();

  for (const Vec3& p : pts) {
    Contact c;
    c.a = A.id;
    c.b = B.id;
    c.point = p;
    c.normal = n;          // B -> A
    c.penetration = penEach;
    out.push_back(c);
  }
}

// ------------------------------------------------------------
// Gather
// ------------------------------------------------------------
void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();
  out.reserve(256);

  for (auto& b : m_bodies) {
    if (!b.isDynamic() || b.asleep) continue;

    if (enableTerrain) contactsBoxTerrain(b, out);
    if (enableGround)  contactsBoxGround(b, out);

    contactsBoxStaticMeshes(b, out);
  }

  for (size_t i = 0; i < m_bodies.size(); ++i) {
    for (size_t j = i + 1; j < m_bodies.size(); ++j) {
      const auto& A = m_bodies[i];
      const auto& B = m_bodies[j];
      if ((!A.isDynamic() && !B.isDynamic()) || (A.asleep && B.asleep)) continue;
      contactsBoxBox(A, B, out);
    }
  }
}

// ------------------------------------------------------------
// Solver
// ------------------------------------------------------------
void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;
  RigidBoxBody* B = (c.b != 0) ? get(c.b) : nullptr;

  Vec3 n = c.normal;
  if (!safeNormalize(n)) return;

  Vec3 ra = c.point - A->position;
  Vec3 va = A->linearVelocity + cross3(A->angularVelocity, ra);

  Vec3 rb{0,0,0};
  Vec3 vb{0,0,0};
  float invMassB = 0.f;
  Mat3 invIBw{};

  if (B) {
    rb = c.point - B->position;
    vb = B->linearVelocity + cross3(B->angularVelocity, rb);
    invMassB = B->invMass;
    invIBw = invInertiaWorld(*B);
  }

  Vec3 rv = va - vb;
  float vn = dot3(rv, n);
  if (vn > 0.f) return;

  Mat3 invIAw = invInertiaWorld(*A);

  Vec3 raXn = cross3(ra, n);
  Vec3 termA = mat3Mul(invIAw, raXn);
  float kA = A->invMass + dot3(cross3(termA, ra), n);

  float kB = 0.f;
  if (B) {
    Vec3 rbXn = cross3(rb, n);
    Vec3 termB = mat3Mul(invIBw, rbXn);
    kB = invMassB + dot3(cross3(termB, rb), n);
  }

  float denom = kA + kB;
  if (!std::isfinite(denom) || denom <= 1e-8f) return;

  const float e = (std::fabs(vn) < 1.0f) ? 0.0f : restitution;
  float j = -(1.f + e) * vn / denom;
  if (!std::isfinite(j)) return;

  if (j > 80.0f) j = 80.0f;

  Vec3 impulse = n * j;

  applyImpulse(*A, impulse, ra);
  if (B) applyImpulse(*B, impulse * -1.f, rb);

  // friction
  rv = (A->linearVelocity + cross3(A->angularVelocity, ra))
     - (B ? (B->linearVelocity + cross3(B->angularVelocity, rb)) : Vec3(0,0,0));

  Vec3 t = rv - n * dot3(rv, n);
  float t2 = len2(t);
  if (t2 > 1e-10f) {
    t = t * (1.f / std::sqrt(t2));

    Vec3 raXt = cross3(ra, t);
    Vec3 termAt = mat3Mul(invIAw, raXt);
    float ktA = A->invMass + dot3(cross3(termAt, ra), t);

    float ktB = 0.f;
    if (B) {
      Vec3 rbXt = cross3(rb, t);
      Vec3 termBt = mat3Mul(invIBw, rbXt);
      ktB = invMassB + dot3(cross3(termBt, rb), t);
    }

    float denomT = ktA + ktB;
    if (denomT > 1e-8f) {
      float jt = -dot3(rv, t) / denomT;

      float maxF = friction * std::max(j, 0.5f);
      jt = clampf(jt, -maxF, maxF);

      Vec3 impT = t * jt;
      applyImpulse(*A, impT, ra);
      if (B) applyImpulse(*B, impT * -1.f, rb);
    }
  }
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;
  RigidBoxBody* B = (c.b != 0) ? get(c.b) : nullptr;

  Vec3 n = c.normal;
  if (!safeNormalize(n)) return;

  const float slop = 0.006f;
  const float percent = 0.18f;

  float pen = c.penetration - slop;
  if (pen <= 0.f) return;

  float wA = A->invMass;
  float wB = B ? B->invMass : 0.f;
  float wSum = wA + wB;
  if (wSum <= 0.f) return;

  Vec3 corr = n * (percent * pen / wSum);

  const float maxCorr = 0.10f;
  float c2 = len2(corr);
  if (c2 > maxCorr * maxCorr) {
    corr = corr * (maxCorr / std::sqrt(c2));
  }

  if (A->isDynamic()) A->position = A->position + corr * wA;
  if (B && B->isDynamic()) B->position = B->position - corr * wB;
}

// ------------------------------------------------------------
// Sleeping
// ------------------------------------------------------------
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

  const float sleepBoost = 1.8f;

  if (lv2 < vThresh2 && av2 < wThresh2) {
    b.sleepTimer += h * sleepBoost;
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

// ------------------------------------------------------------
// Player collision
// ------------------------------------------------------------
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
    Vec3 cl(
      clampf(local.x, -b.halfExtents.x, b.halfExtents.x),
      clampf(local.y, -b.halfExtents.y, b.halfExtents.y),
      clampf(local.z, -b.halfExtents.z, b.halfExtents.z)
    );

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

    // push box horizontally only (no rocket up)
    Vec3 nh(nRaw.x, 0.0f, nRaw.z);
    if (safeNormalize(nh)) {
      if (std::fabs(nRaw.y) < 0.85f) {
        float vInto = -dot3(playerVelIn, nh);
        if (vInto > 0.f) {
          float j = 70.0f * vInto;
          if (j > 22.0f) j = 22.0f;
          applyImpulse(b, nh * (-j), Vec3(0,0,0));
        }
      }
    }

    if (nRaw.y > 0.6f) grounded = true;
    hit = true;
  }

  if (enableTerrain && m_terrainHeightFn) {
    const float h = terrainHeightAt(center.x, center.z);
    Vec3 n = terrainNormalAt(center.x, center.z);
    if (!safeNormalize(n)) n = Vec3(0,1,0);

    Vec3 p(center.x, h, center.z);
    float dist = dot3(center - p, n);
    float pen = radius - dist;
    if (pen > 0.f) {
      center = center + n * pen;
      hit = true;
      if (n.y > 0.75f) grounded = true;

      float vn = dot3(playerVel, n);
      if (vn < 0.f) playerVel = playerVel - n * vn;
    }
  }

  if (enableGround) {
    float bottom = center.y - radius;
    if (bottom < groundY) {
      float pen = groundY - bottom;
      center.y += pen;
      hit = true;
      grounded = true;
      if (playerVel.y < 0.f) playerVel.y = 0.f;
    }
  }

  if (outGrounded) *outGrounded = grounded;
  return hit;
}

// ------------------------------------------------------------
// Step
// ------------------------------------------------------------
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

  std::vector<uint8_t> supported(m_bodies.size(), 0);
  for (const auto& c : contacts) {
    if (c.normal.y < 0.75f) continue;
    for (size_t i=0;i<m_bodies.size();++i) {
      if (m_bodies[i].id == c.a) { supported[i] = 1; break; }
    }
  }

  for (size_t i=0;i<m_bodies.size();++i)
    updateSleeping(m_bodies[i], h, supported[i] != 0);
}

void PhysicsWorldRB::step(float dt) {
  m_accum += dt;
  while (m_accum >= fixedDt) {
    substep(fixedDt);
    m_accum -= fixedDt;
  }
}

} // namespace fe
