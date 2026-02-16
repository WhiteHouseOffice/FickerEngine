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
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b ? b : x); }

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

// Small “don’t jitter when resting” thresholds
static constexpr float kSupportNMin      = 0.75f;  // normal.y must be >= this to count as support
static constexpr float kSupportSkin      = 0.06f;  // allow near-contact
static constexpr float kRestLinVel       = 0.25f;  // below => we can damp
static constexpr float kRestAngVel       = 1.25f;  // below => we can damp
static constexpr float kRestDampLin      = 0.55f;  // how strongly we kill small lateral drift per substep
static constexpr float kRestDampAng      = 0.55f;  // how strongly we kill small ang drift per substep
static constexpr float kMaxPlayerPushJ   = 22.0f;  // walking push cap
static constexpr float kPlayerPushScale  = 70.0f;  // walking push strength

// ============================================================
// Simple 2D convex hull + point-in-convex (XZ plane)
// ============================================================
struct V2 { float x, z; };

static inline float cross2(const V2& a, const V2& b, const V2& c) {
  // (b-a) x (c-a)
  return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

static void convexHullXZ(const std::vector<V2>& pts, std::vector<V2>& outHull) {
  outHull.clear();
  if (pts.size() < 3) return;

  // Monotonic chain
  std::vector<V2> p = pts;
  std::sort(p.begin(), p.end(), [](const V2& a, const V2& b){
    if (a.x != b.x) return a.x < b.x;
    return a.z < b.z;
  });

  std::vector<V2> lower, upper;
  for (const auto& v : p) {
    while (lower.size() >= 2 && cross2(lower[lower.size()-2], lower[lower.size()-1], v) <= 0.f)
      lower.pop_back();
    lower.push_back(v);
  }
  for (int i=(int)p.size()-1; i>=0; --i) {
    const auto& v = p[(size_t)i];
    while (upper.size() >= 2 && cross2(upper[upper.size()-2], upper[upper.size()-1], v) <= 0.f)
      upper.pop_back();
    upper.push_back(v);
  }

  // Concatenate (last point of each is duplicate)
  lower.pop_back();
  upper.pop_back();
  outHull = lower;
  outHull.insert(outHull.end(), upper.begin(), upper.end());
}

static bool pointInConvexXZ(const std::vector<V2>& hull, const V2& p) {
  if (hull.size() < 3) return false;
  // All cross products should have same sign for convex polygon (assumed CCW)
  float prev = 0.f;
  for (size_t i=0;i<hull.size();++i) {
    const V2& a = hull[i];
    const V2& b = hull[(i+1) % hull.size()];
    float c = cross2(a, b, p);
    if (i==0) prev = c;
    else {
      if ((prev >= 0.f && c < 0.f) || (prev < 0.f && c >= 0.f))
        return false;
    }
  }
  return true;
}

// ============================================================
// OBB helpers
// ============================================================
static void computeOBBAxes(const RigidBoxBody& b, Vec3& ax, Vec3& ay, Vec3& az) {
  Quat qn = safeQuatNormalize(b.orientation);
  Mat3 R = quatToMat3(qn);
  ax = Vec3(R.m[0][0], R.m[1][0], R.m[2][0]);
  ay = Vec3(R.m[0][1], R.m[1][1], R.m[2][1]);
  az = Vec3(R.m[0][2], R.m[1][2], R.m[2][2]);
}

static Vec3 obbVertex(const RigidBoxBody& b, int sx, int sy, int sz) {
  Vec3 ax, ay, az;
  computeOBBAxes(b, ax, ay, az);
  return b.position
    + ax * (b.halfExtents.x * (float)sx)
    + ay * (b.halfExtents.y * (float)sy)
    + az * (b.halfExtents.z * (float)sz);
}

static inline float absf(float x){ return x < 0.f ? -x : x; }

// ============================================================
// World inertia helpers
// ============================================================
Mat3 PhysicsWorldRB::invInertiaWorld(const RigidBoxBody& b) const {
  Quat qn = safeQuatNormalize(b.orientation);
  Mat3 R = quatToMat3(qn);
  Mat3 Rt = mat3Transpose(R);
  // Iw^-1 = R * Ilocal^-1 * R^T
  return mat3Mul(mat3Mul(R, b.invInertiaLocal), Rt);
}

// ============================================================
// Create / access
// ============================================================
uint32_t PhysicsWorldRB::createBox(const Vec3& pos, const Vec3& halfExtents, float mass) {
  RigidBoxBody b;
  b.id = m_nextId++;
  b.position = pos;
  b.orientation = Quat::identity();
  b.linearVelocity = Vec3(0,0,0);
  b.angularVelocity = Vec3(0,0,0);
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
  for (auto& b : m_bodies) if (b.id == id) return &b;
  return nullptr;
}

void PhysicsWorldRB::clearDynamics() {
  m_bodies.clear();
  m_nextId = 1;
}

// ============================================================
// Integration + damping + sleeping
// ============================================================
void PhysicsWorldRB::applyDamping(RigidBoxBody& b, float h) {
  if (!b.isDynamic() || b.asleep) return;

  // Stable-ish damping: v *= 1/(1 + d*dt)
  const float ld = std::max(0.f, b.linearDamping);
  const float ad = std::max(0.f, b.angularDamping);

  const float linMul = 1.f / (1.f + ld * h);
  const float angMul = 1.f / (1.f + ad * h);

  b.linearVelocity = b.linearVelocity * linMul;
  b.angularVelocity = b.angularVelocity * angMul;
}

void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h) {
  if (!b.isDynamic()) return;

  // If it is basically not moving, accumulate sleep time.
  const float lv2 = len2(b.linearVelocity);
  const float av2 = len2(b.angularVelocity);

  const float vThresh2  = b.sleepVel * b.sleepVel;
  const float wThresh2  = b.sleepAngVel * b.sleepAngVel;

  if (lv2 < vThresh2 && av2 < wThresh2) {
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

void PhysicsWorldRB::applyImpulse(RigidBoxBody& b, const Vec3& impulse, const Vec3& r) {
  if (!b.isDynamic()) return;

  // Wake on meaningful impulse
  if (b.asleep && len2(impulse) > 1e-6f) {
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

// ============================================================
// Contacts
// ============================================================
void PhysicsWorldRB::gatherContacts(std::vector<Contact>& out) {
  out.clear();

  // dynamic vs ground / statics
  for (const auto& a : m_bodies) {
    if (!a.isDynamic() || a.asleep) continue;

    if (enableGround) contactsBoxGround(a, out);
    for (const auto& s : m_static) contactsBoxStaticAABB(a, s, out);
  }

  // dynamic vs dynamic
  const size_t n = m_bodies.size();
  for (size_t i=0;i<n;++i) {
    const auto& A = m_bodies[i];
    if (!A.isDynamic() || A.asleep) continue;

    for (size_t j=i+1;j<n;++j) {
      const auto& B = m_bodies[j];
      if (!B.isDynamic() || B.asleep) continue;

      contactsBoxBox(A, B, out);
    }
  }
}

void PhysicsWorldRB::contactsBoxGround(const RigidBoxBody& A, std::vector<Contact>& out) {
  // Emit contacts when within a small "skin" above the plane too.
  const float skin = std::max(contactSkin, kSupportSkin);

  float minY = 1e30f;
  Vec3 verts[8];
  int idx = 0;

  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s) {
    Vec3 v = obbVertex(A, ix, iy, iz);
    verts[idx++] = v;
    minY = std::min(minY, v.y);
  }

  // If the lowest vertex is well above ground+skin, no contact.
  if (minY > groundY + skin) return;

  // Pick up to 4 points near the bottom-most area.
  // This gives stable resting on flat faces, while still allowing corner contacts for tipping.
  int emitted = 0;

  // Threshold: either near minY, or close to ground plane within skin.
  const float nearMin = minY + 0.02f;
  const float planeBand = groundY + skin;

  for (int i=0;i<8 && emitted<4;i++) {
    const float y = verts[i].y;
    if (y <= nearMin && y <= planeBand) {
      Contact c;
      c.a = A.id;
      c.b = 0;
      c.point = verts[i];
      c.normal = Vec3(0,1,0);

      // penetration is clamped >= 0 (skin contacts can be 0)
      c.penetration = std::max(0.f, groundY - y);

      out.push_back(c);
      emitted++;
    }
  }

  // Fallback: if we didn't emit anything (rare numeric corner cases), emit the minY vertex
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
  // Top support manifold with edge tolerance + clamped point
  const float topY = S.max.y;

  Vec3 ax, ay, az;
  computeOBBAxes(A, ax, ay, az);

  // Broad: AABB overlap test in world space by OBB extents
  // Cheap approximation: OBB projected extents along world axes
  Vec3 ex(
    absf(ax.x)*A.halfExtents.x + absf(ay.x)*A.halfExtents.y + absf(az.x)*A.halfExtents.z,
    absf(ax.y)*A.halfExtents.x + absf(ay.y)*A.halfExtents.y + absf(az.y)*A.halfExtents.z,
    absf(ax.z)*A.halfExtents.x + absf(ay.z)*A.halfExtents.y + absf(az.z)*A.halfExtents.z
  );

  Vec3 aMin = A.position - ex;
  Vec3 aMax = A.position + ex;

  if (aMax.x < S.min.x || aMin.x > S.max.x) return;
  if (aMax.z < S.min.z || aMin.z > S.max.z) return;

  // If OBB is not near the top plane, bail
  const float skin = std::max(contactSkin, kSupportSkin);
  if (aMin.y > topY + skin) return;
  if (aMax.y < topY - 3.0f) return;

  // Use bottom-ish vertices as contact candidates
  Vec3 verts[8];
  int idx = 0;

  float bestY = 1e30f;
  const int sgn[2] = {-1,1};
  for (int ix : sgn) for (int iy : sgn) for (int iz : sgn) {
    Vec3 v = obbVertex(A, ix, iy, iz);
    verts[idx++] = v;
    bestY = std::min(bestY, v.y);
  }

  // Emit up to 4 contacts that are within footprint and near the top plane
  int emitted = 0;
  for (int i=0;i<8 && emitted<4;i++) {
    const Vec3& v = verts[i];

    // inside static top footprint
    if (v.x < S.min.x - 0.02f || v.x > S.max.x + 0.02f) continue;
    if (v.z < S.min.z - 0.02f || v.z > S.max.z + 0.02f) continue;

    // near the platform top band (allow skin above)
    if (v.y > topY + skin) continue;

    // also prefer bottom-most vertices (stable)
    if (v.y > bestY + 0.03f) continue;

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

void PhysicsWorldRB::contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out) {
  // Minimal (kept from your prior version): treat as broad overlap via world AABB from projected extents.
  Vec3 axA, ayA, azA; computeOBBAxes(A, axA, ayA, azA);
  Vec3 axB, ayB, azB; computeOBBAxes(B, axB, ayB, azB);

  Vec3 exA(
    absf(axA.x)*A.halfExtents.x + absf(ayA.x)*A.halfExtents.y + absf(azA.x)*A.halfExtents.z,
    absf(axA.y)*A.halfExtents.x + absf(ayA.y)*A.halfExtents.y + absf(azA.y)*A.halfExtents.z,
    absf(axA.z)*A.halfExtents.x + absf(ayA.z)*A.halfExtents.y + absf(azA.z)*A.halfExtents.z
  );
  Vec3 exB(
    absf(axB.x)*B.halfExtents.x + absf(ayB.x)*B.halfExtents.y + absf(azB.x)*B.halfExtents.z,
    absf(axB.y)*B.halfExtents.x + absf(ayB.y)*B.halfExtents.y + absf(azB.y)*B.halfExtents.z,
    absf(axB.z)*B.halfExtents.x + absf(ayB.z)*B.halfExtents.y + absf(azB.z)*B.halfExtents.z
  );

  Vec3 aMin = A.position - exA;
  Vec3 aMax = A.position + exA;
  Vec3 bMin = B.position - exB;
  Vec3 bMax = B.position + exB;

  if (aMax.x < bMin.x || aMin.x > bMax.x) return;
  if (aMax.y < bMin.y || aMin.y > bMax.y) return;
  if (aMax.z < bMin.z || aMin.z > bMax.z) return;

  // Create a single approximate contact at midpoint with upward-ish normal.
  // (Good enough for now; later we can upgrade to SAT + manifolds.)
  Vec3 mid = (A.position + B.position) * 0.5f;
  Vec3 n = A.position - B.position;
  if (!safeNormalize(n)) n = Vec3(0,1,0);

  Contact c;
  c.a = A.id;
  c.b = B.id;
  c.point = mid;
  c.normal = n; // from A toward B
  c.penetration = 0.02f; // small bias to keep them from sinking into each other
  out.push_back(c);
}

// ============================================================
// Solver
// ============================================================
void PhysicsWorldRB::solveVelocity(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;

  const bool hasB = (c.b != 0);
  RigidBoxBody* B = hasB ? get(c.b) : nullptr;

  Vec3 n = c.normal;
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

  const float maxJ = 80.0f;
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

      // Ensure friction exists even on tiny normal impulses (skin contacts)
      float maxF = friction * std::max(j, 0.5f);
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

  const float maxCorr = 0.15f;
  float c2 = len2(corr);
  if (c2 > maxCorr * maxCorr) {
    corr = corr * (maxCorr / std::sqrt(c2));
  }

  if (A->isDynamic()) A->position = A->position + corr * wA;
  if (B && B->isDynamic()) B->position = B->position - corr * wB;
}

// ============================================================
// COM-support stabilization pass (the “upgrade”)
// ============================================================
static void stabilizeSupportedBodies(std::vector<RigidBoxBody>& bodies,
                                    const std::vector<Contact>& contacts,
                                    const std::vector<AABB>& statics) {
  const size_t n = bodies.size();
  if (n == 0) return;

  std::vector<uint8_t> supported(n, 0);

  // Collect support points (XZ) per body
  std::vector<std::vector<V2>> supportPts(n);

  auto findIndexById = [&](uint32_t id) -> int {
    for (int i=0;i<(int)bodies.size();++i) if (bodies[(size_t)i].id == id) return i;
    return -1;
  };

  // 1) Contacts-based support (ground, static-top, box-on-box)
  for (const auto& c : contacts) {
    if (c.normal.y < kSupportNMin) continue;
    if (c.penetration < -kSupportSkin) continue;

    int ia = findIndexById(c.a);
    if (ia < 0) continue;

    supportPts[(size_t)ia].push_back(V2{c.point.x, c.point.z});
  }

  // 2) Determine support by COM-in-support-polygon (or static footprint as a fallback)
  for (size_t i=0;i<n;++i) {
    const auto& b = bodies[i];
    if (!b.isDynamic()) continue;

    const V2 com{ b.position.x, b.position.z };

    if (supportPts[i].size() >= 3) {
      std::vector<V2> hull;
      convexHullXZ(supportPts[i], hull);
      if (pointInConvexXZ(hull, com)) supported[i] = 1;
    }

    if (!supported[i]) {
      for (const auto& s : statics) {
        if (com.x >= s.min.x && com.x <= s.max.x &&
            com.z >= s.min.z && com.z <= s.max.z) {
          if (b.position.y >= s.max.y - 2.0f && b.position.y <= s.max.y + 2.0f) {
            if (b.linearVelocity.y <= 0.5f) {
              supported[i] = 1;
              break;
            }
          }
        }
      }
    }
  }

  // 3) Stabilize: kill tiny lateral/ang jitter while supported
  for (size_t i=0;i<n;++i) {
    if (!supported[i]) continue;
    auto& b = bodies[i];
    if (!b.isDynamic()) continue;

    Vec3 lv = b.linearVelocity;
    Vec3 av = b.angularVelocity;

    const float av2 = len2(av);

    Vec3 lateral(lv.x, 0.f, lv.z);
    float lat2 = len2(lateral);

    if (lat2 < kRestLinVel*kRestLinVel) {
      b.linearVelocity.x *= (1.f - kRestDampLin);
      b.linearVelocity.z *= (1.f - kRestDampLin);
    }

    if (av2 < kRestAngVel*kRestAngVel) {
      b.angularVelocity.x *= (1.f - kRestDampAng);
      b.angularVelocity.y *= (1.f - kRestDampAng);
      b.angularVelocity.z *= (1.f - kRestDampAng);
    }
  }
}

// ============================================================
// Player collision (kept from your version)
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
    if (d2 > radius*radius) continue;

    float d = (d2 > 1e-10f) ? std::sqrt(d2) : 0.f;
    Vec3 n = (d > 1e-6f) ? (delta * (1.f/d)) : Vec3(0,1,0);

    float pen = radius - d;
    if (pen < 0.f) continue;

    // Position correction
    center = center + n * pen;
    hit = true;

    if (n.y >= kSupportNMin) grounded = true;

    // Player "push" impulse onto body (horizontal-only; avoid rocket)
    Vec3 pushDir = Vec3(playerVelIn.x, 0.f, playerVelIn.z);
    if (safeNormalize(pushDir)) {
      float desired = std::min(kMaxPlayerPushJ, kPlayerPushScale * pen);
      Vec3 j = pushDir * desired;
      applyImpulse(b, j, closest - b.position);
    }

    // Remove velocity into the object
    float vn = dot3(playerVel, n);
    if (vn < 0.f) playerVel = playerVel - n * vn;
  }

  // Ground plane for player sphere
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

// ============================================================
// Step
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

  // Stabilize supported bodies (reduces micro-drift/jitter)
  stabilizeSupportedBodies(m_bodies, contacts, m_static);

  // Sleeping pass (after corrections + stabilization)
  for (auto& b : m_bodies) updateSleeping(b, h);
}

void PhysicsWorldRB::step(float dt) {
  m_accum += dt;
  while (m_accum >= fixedDt) {
    substep(fixedDt);
    m_accum -= fixedDt;
  }
}

} // namespace fe
