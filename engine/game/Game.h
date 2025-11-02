#pragma once

#if __has_include("math/MiniMath.h")
  #include "math/MiniMath.h"
#else
  struct Vec3 { float x,y,z; Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
                              Vec3& operator+=(const Vec3& o){ x+=o.x;y+=o.y;z+=o.z; return *this; }
                              Vec3 operator*(float s) const { return {x*s,y*s,z*s}; } };
  struct Mat4 { float m[16]; };
  Mat4 lookAt(const Vec3& eye, const Vec3& at, const Vec3& up);
#endif

class Game {
public:
    void Init();
    void Update(float dt);
    Mat4 View() const;

private:
    Vec3  camPos { 0.f, 2.f, 5.f };
    float yaw    = 0.0f;
    float pitch  = 0.0f;
    float moveSpeed = 5.0f;
    float logTimer  = 0.0f;
};
