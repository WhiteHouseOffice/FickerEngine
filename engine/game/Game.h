#pragma once
#include "math/MiniMath.h"   // for Vec3, Mat4

class Game {
public:
    void Init();
    void Update(float dt);
    Mat4 View() const;

private:
    Vec3  camPos { 0.f, 2.f, 5.f };
    float yaw    = 0.0f;          // radians
    float pitch  = 0.0f;          // reserved
    float moveSpeed = 5.0f;
    float logTimer  = 0.0f;
};
