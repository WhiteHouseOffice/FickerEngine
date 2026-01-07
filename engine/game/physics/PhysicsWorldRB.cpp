#include "game/physics/PhysicsWorldRB.h"
#include <cmath>
#include <algorithm>

namespace fe {

static inline float dot3(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float len2(const Vec3& v){ return dot3(v,v); }

static inline Vec3 clampVec3(const Vec3& v, const Vec3& mn, const Vec3& mx) {
  return Vec3(
    (v.x < mn.x) ? mn.x : (v.x > mx.x ? mx.x : v.x),
    (v.y < mn.y) ? mn.y : (v.y > mx.y ? mx.y : v.y),
    (v.z < mn.z) ? mn.z : (v.z > mx.z ? mx.z : v.z)
  );
}

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
  for (auto& b : m_bodies) if (b.id == id) return &b;
  return nullptr;
}

void PhysicsWorldRB::clearDynamics() {
  m_bodies.clear();
  m_nextId = 1;
  m_accum = 0.f;
}

Mat3 PhysicsWorldRB::invInertiaWorld(const RigidBoxBody& b) const {
  // R * I^-1_local * R^T
  const Quat qn = quatNormalize(b.orientation);
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
}

void PhysicsWorldRB::integrateOrientation(RigidBoxBody& b, float h) {
  // q' = 0.5 * omegaQuat * q
  Quat q = quatNormalize(b.orientation);
  const Vec3 w = b.angularVelocity;
  Quat omega{0.f, w.x, w.y, w.z};
  Quat dq = quatMul(omega, q);
  q.w += 0.5f * dq.w * h;
  q.x += 0.5f * dq.x * h;
  q.y += 0.5f * dq.y * h;
  q.z += 0.5f * dq.z * h;
  b.orientation = quatNormalize(q);
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
}

static void computeOBBAxes(const RigidBoxBody& b, Vec3& ax, Vec3& ay, Vec3& az) {
  const Quat qn = quatNormalize(b.orientation);
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
    if (l2 < 1e-10f) continue;
    float invL = 1.f / std::sqrt(l2);
    a = a * invL;

    float ra = projectRadius(A, a);
    float rb = projectRadius(B, a);
    float dist = std::fabs(dot3(d, a));
    float pen = (ra + rb) - dist;
    if (pen < 0.f) return false;

    if (pen < bestPen) {
      bestPen = pen;
      // make axis point from A to B
      bestAxis = (dot3(d, a) < 0.f) ? (a * -1.f) : a;
    }
  }

  outN = bestAxis;
  outPen = bestPen;
  return true;
}

void PhysicsWorldRB::contactsBoxBox(const RigidBoxBody& A, const RigidBoxBody& B, std::vector<Contact>& out) {
  Vec3 n; float pen;
  if (!satOBBOBB(A,B,n,pen)) return;

  // Collect contact points by vertex inclusion (gives multiple points for face contacts)
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

  // Fallback: single point midway between centers projected along normal
  if (pts.empty()) {
    Vec3 p = (A.position + B.position) * 0.5f - n * (pen * 0.5f);
    pts.push_back(p);
  }

  // Reduce to at most 4 points (fast)
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
  // If lowest vertex below ground -> contact
  float minY = 1e30f;
  Vec3 minP = A.position;
  const int s[2]={-1,1};
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(A, ix,iy,iz);
    if (v.y < minY) { minY=v.y; minP=v; }
  }
  if (minY < groundY) {
    Contact c;
    c.a = A.id; c.b = 0;
    c.point = minP;
    c.normal = Vec3(0,1,0);
    c.penetration = (groundY - minY);
    out.push_back(c);
  }
}

void PhysicsWorldRB::contactsBoxStaticAABB(const RigidBoxBody& A, const AABB& S, std::vector<Contact>& out) {
  // Approximate OBB vs AABB via sampling vertices against AABB
  const int s[2]={-1,1};
  for (int ix: s) for (int iy: s) for (int iz: s) {
    Vec3 v = obbVertex(A, ix,iy,iz);
    Vec3 cl = clampVec3(v, S.min, S.max);
    Vec3 d = v - cl;
    float d2 = len2(d);
    if (d2 <= 1e-10f) {
      // vertex is inside AABB => push out along best axis
      // compute penetration depth to each face
      float px = std::min(v.x - S.min.x, S.max.x - v.x);
      float py = std::min(v.y - S.min.y, S.max.y - v.y);
      float pz = std::min(v.z - S.min.z, S.max.z - v.z);
      Vec3 n(0,1,0);
      float pen = py;
      if (px < pen) { pen=px; n = (v.x - (S.min.x+S.max.x)*0.5f) < 0 ? Vec3(-1,0,0) : Vec3(1,0,0); }
      if (pz < pen) { pen=pz; n = (v.z - (S.min.z+S.max.z)*0.5f) < 0 ? Vec3(0,0,-1) : Vec3(0,0,1); }
      Contact c;
      c.a = A.id; c.b = 0;
      c.point = v;
      c.normal = n;
      c.penetration = pen;
      out.push_back(c);
      if (out.size() >= 4) break;
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
const float n2 = dot3(n, n);
if (n2 < 1e-12f) {
  // Degenerate contact → skip
  return;
}
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
  if (vn > 0.f) return; // separating

  // effective mass along normal
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
  if (denom <= 1e-8f) return;

 const float eps = 1e-8f;
if (std::fabs(denom) < eps) return;

float j = -(1.f + restitution) * vn / denom;

  Vec3 impulse = n * j;
  applyImpulse(*A, impulse * -1.f, ra);
  if (B) applyImpulse(*B, impulse, rb);

  // friction (Coulomb)
  rv = (A->linearVelocity + cross(A->angularVelocity, ra)) - (B ? (B->linearVelocity + cross(B->angularVelocity, rb)) : Vec3(0,0,0));
  Vec3 t = rv - n * dot3(rv, n);
  float t2 = len2(t);
  if (t2 > 1e-10f) {
    float invT = 1.f / std::sqrt(t2);
    t = t * invT;

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
      float maxF = friction * j;
      if (jt >  maxF) jt =  maxF;
      if (jt < -maxF) jt = -maxF;

      Vec3 impT = t * jt;
      applyImpulse(*A, impT * -1.f, ra);
      if (B) applyImpulse(*B, impT, rb);
    }
  }

  // wake on contact
  A->asleep = false;
  A->sleepTimer = 0.f;
  if (B) { B->asleep=false; B->sleepTimer=0.f; }
}

void PhysicsWorldRB::solvePosition(const Contact& c) {
  RigidBoxBody* A = get(c.a);
  if (!A) return;
  RigidBoxBody* B = (c.b != 0) ? get(c.b) : nullptr;

  const float slop = 0.002f;
  const float percent = 0.8f;
  float pen = std::max(0.f, c.penetration - slop);
  if (pen <= 0.f) return;

  const Vec3 n = c.normal;
  float wA = A->invMass;
  float wB = B ? B->invMass : 0.f;
  float wSum = wA + wB;
  if (wSum <= 0.f) return;

  Vec3 corr = n * (percent * pen / wSum);
  if (A->isDynamic()) A->position = A->position - corr * wA;
  if (B && B->isDynamic()) B->position = B->position + corr * wB;
}

void PhysicsWorldRB::updateSleeping(RigidBoxBody& b, float h) {
  if (!b.isDynamic()) return;

  const float v2 = len2(b.linearVelocity);
  const float w2 = len2(b.angularVelocity);

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

  // collide vs dynamic boxes: sphere vs OBB (closest point in OBB)
  for (auto& b : m_bodies) {
    if (!b.isDynamic()) continue;

    const Quat qn = quatNormalize(b.orientation);
    const Mat3 R = quatToMat3(qn);
    const Mat3 Rt = mat3Transpose(R);

    // sphere center in box local
    Vec3 d = center - b.position;
    Vec3 local = mat3Mul(Rt, d);

    Vec3 cl = clampVec3(local, Vec3(-b.halfExtents.x, -b.halfExtents.y, -b.halfExtents.z),
                               Vec3( b.halfExtents.x,  b.halfExtents.y,  b.halfExtents.z));
    Vec3 closest = mat3Mul(R, cl) + b.position;
    Vec3 delta = center - closest;
    float d2 = len2(delta);
    if (d2 < radius*radius && d2 > 1e-10f) {
      float dist = std::sqrt(d2);
      Vec3 n = delta * (1.f/dist);
      float pen = radius - dist;

      // move player out
      center = center + n * pen;
      // remove velocity into normal
      float vn = dot3(playerVel, n);
      if (vn < 0.f) playerVel = playerVel - n * vn;

      // push box (simple impulse based on player velocity)
      // treat player as heavy kinematic: apply impulse to box opposite n
      float push = 6.0f; // tune
      b.linearVelocity = b.linearVelocity + (n * (-push * vn)) * b.invMass;

      if (n.y > 0.6f) grounded = true;
      hit = true;
      b.asleep = false;
      b.sleepTimer = 0.f;
    }
  }

  // collide vs static AABBs
  for (const auto& s : m_static) {
    Vec3 cl = clampVec3(center, s.min, s.max);
    Vec3 d = center - cl;
    float d2 = len2(d);
    if (d2 < radius*radius) {
      // push out along best axis
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

  // Solve velocities
  for (int it=0; it<velocityIters; ++it) {
    for (const auto& c : contacts) solveVelocity(c);
  }

  // Positional correction
  for (int it=0; it<positionIters; ++it) {
    for (const auto& c : contacts) solvePosition(c);
  }

  for (auto& b : m_bodies) updateSleeping(b, h);
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
