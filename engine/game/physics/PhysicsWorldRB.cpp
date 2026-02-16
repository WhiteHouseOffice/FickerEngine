#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>

namespace fe {

// ======================
// Math helpers
// ======================
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

// ======================
// Closest point triangle
// ======================
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

// ======================
// OBB helpers
// ======================
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

// ======================
// BOX vs BOX (RESTORED)
// ======================
void PhysicsWorldRB::contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out) {
  Vec3 n{0,1,0};
  float pen = 0.01f;

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
    Vec3 p = (A.position + B.position) * 0.5f;
    pts.push_back(p);
  }

  if (pts.size() > 4) pts.resize(4);
  const float penEach = pen / (float)pts.size();

  for (const Vec3& p : pts) {
    Contact c;
    c.a = A.id;
    c.b = B.id;
    c.point = p;
    c.normal = n;
    c.penetration = penEach;
    out.push_back(c);
  }
}

// ======================
// STATIC MESH CONTACT
// ======================
void PhysicsWorldRB::contactsBoxStaticMeshes(const RigidBoxBody& A, std::vector<Contact>& out) {
  if (m_staticMeshes.empty()) return;

  Vec3 boxVerts[8];
  int idx = 0;
  const int s[2] = {-1,1};
  for (int ix : s) for (int iy : s) for (int iz : s)
    boxVerts[idx++] = obbVertex(A, ix, iy, iz);

  for (int vi = 0; vi < 8; ++vi) {
    const Vec3& p = boxVerts[vi];

    for (const auto& mesh : m_staticMeshes) {
      for (size_t ti = 0; ti + 2 < mesh.indices.size(); ti += 3) {
        const Vec3& a = mesh.verts[mesh.indices[ti+0]];
        const Vec3& b = mesh.verts[mesh.indices[ti+1]];
        const Vec3& c = mesh.verts[mesh.indices[ti+2]];

        Vec3 cp = closestPointOnTri(p, a, b, c);
        Vec3 n = cross3(b - a, c - a);
        if (!safeNormalize(n)) continue;

        float dist = dot3(p - cp, n);
        if (dist > 0.02f) continue;

        Contact cc;
        cc.a = A.id;
        cc.b = 0;
        cc.point = cp;
        cc.normal = n;
        cc.penetration = 0.02f - dist;
        out.push_back(cc);
      }
    }
  }
}

} // namespace fe
