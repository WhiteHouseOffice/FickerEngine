#pragma once
#include "math/MiniMath.h"

class Game {
public:
    void Init();
    void Update(float dt);
    Mat4 View() const;

private:
    Vec3 camPos { 0.f, 2.f, 5.f };
    float yaw   = 0.0f;   // radians, Y-axis
    float pitch = 0.0f;   // radians, X-axis (unused for now; keep 0)
    float moveSpeed = 5.0f;
    float logTimer  = 0.0f;
};
