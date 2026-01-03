#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include "math/MiniMath.h"

namespace fe {

// Minimal quaternion for rigid body orientation
struct Quat {
  float w=1.f, x=0.f, y=0.f, z=0.f;

  static Quat identity() { return Quat{1,0,0,0}; }

  static Quat fromAxisAngle(const Vec3& axis, float radians) {
    Vec3 a = normalize(axis);
    const float half = 0.5f * radians;
    const float s = std::sin(half);
    return Quat{std::cos(half), a.x*s, a.y*s, a.z*s};
  }
};

inline Quat quatNormalize(const Quat& q) {
  const float n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n <= 0.f) return Quat::identity();
  const float inv = 1.f / n;
  return Quat{q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

inline Quat quatMul(const Quat& a, const Quat& b) {
  return Quat{
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

inline Vec3 quatRotate(const Quat& qn, const Vec3& v) {
  // v' = q * (0,v) * q^-1 ; q assumed normalized
  Vec3 qv{qn.x, qn.y, qn.z};
  Vec3 t = cross(qv, v) * 2.0f;
  return v + t * qn.w + cross(qv, t);
}

struct Mat3 {
  float m[3][3]{};
};

inline Mat3 quatToMat3(const Quat& qn) {
  const float w=qn.w,x=qn.x,y=qn.y,z=qn.z;
  Mat3 R{};
  const float xx=x*x, yy=y*y, zz=z*z;
  const float xy=x*y, xz=x*z, yz=y*z;
  const float wx=w*x, wy=w*y, wz=w*z;

  R.m[0][0] = 1.f - 2.f*(yy+zz);
  R.m[0][1] = 2.f*(xy - wz);
  R.m[0][2] = 2.f*(xz + wy);

  R.m[1][0] = 2.f*(xy + wz);
  R.m[1][1] = 1.f - 2.f*(xx+zz);
  R.m[1][2] = 2.f*(yz - wx);

  R.m[2][0] = 2.f*(xz - wy);
  R.m[2][1] = 2.f*(yz + wx);
  R.m[2][2] = 1.f - 2.f*(xx+yy);
  return R;
}

inline Vec3 mat3Mul(const Mat3& A, const Vec3& v) {
  return Vec3(
    A.m[0][0]*v.x + A.m[0][1]*v.y + A.m[0][2]*v.z,
    A.m[1][0]*v.x + A.m[1][1]*v.y + A.m[1][2]*v.z,
    A.m[2][0]*v.x + A.m[2][1]*v.y + A.m[2][2]*v.z
  );
}

inline Mat3 mat3Transpose(const Mat3& A) {
  Mat3 T{};
  for (int r=0;r<3;++r) for(int c=0;c<3;++c) T.m[r][c]=A.m[c][r];
  return T;
}

inline Mat3 mat3Mul(const Mat3& A, const Mat3& B) {
  Mat3 C{};
  for(int r=0;r<3;++r) for(int c=0;c<3;++c){
    C.m[r][c]=0.f;
    for(int k=0;k<3;++k) C.m[r][c]+=A.m[r][k]*B.m[k][c];
  }
  return C;
}

// Axis-aligned box used as static collider (world/platforms)
struct AABB {
  Vec3 min{0.f,0.f,0.f};
  Vec3 max{0.f,0.f,0.f};
};

// Dynamic oriented box rigid body
struct RigidBoxBody {
  uint32_t id = 0;

  Vec3 position{0.f,0.f,0.f};
  Quat orientation = Quat::identity();

  Vec3 linearVelocity{0.f,0.f,0.f};
  Vec3 angularVelocity{0.f,0.f,0.f}; // radians/sec in world space

  Vec3 halfExtents{0.5f,0.5f,0.5f};

  float invMass = 0.f;     // 0 => static/kinematic
  Mat3  invInertiaLocal{}; // in local/body space

  float linearDamping = 0.02f;
  float angularDamping = 0.05f;

  bool useGravity = true;

  // sleeping
  bool  asleep = false;
  float sleepTimer = 0.f;
  float sleepVel = 0.08f;     // m/s
  float sleepAngVel = 0.15f;  // rad/s
  float sleepTime = 0.5f;     // seconds

  bool isDynamic() const { return invMass > 0.f; }
};

inline Mat3 boxInertiaInvLocal(const Vec3& he, float mass) {
  // Inertia tensor for box about center:
  // Ixx = (1/12)m( (2hy)^2 + (2hz)^2 ) = (1/3)m( hy^2 + hz^2 )
  // and similarly.
  Mat3 inv{};
  if (mass <= 0.f) return inv;
  const float hx=he.x, hy=he.y, hz=he.z;
  const float Ixx = (1.f/3.f) * mass * (hy*hy + hz*hz);
  const float Iyy = (1.f/3.f) * mass * (hx*hx + hz*hz);
  const float Izz = (1.f/3.f) * mass * (hx*hx + hy*hy);
  inv.m[0][0] = (Ixx > 0.f) ? 1.f/Ixx : 0.f;
  inv.m[1][1] = (Iyy > 0.f) ? 1.f/Iyy : 0.f;
  inv.m[2][2] = (Izz > 0.f) ? 1.f/Izz : 0.f;
  return inv;
}

} // namespace fe
