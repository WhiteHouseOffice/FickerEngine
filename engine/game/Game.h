#pragma once

#include "math/MiniMath.h"  // Vec3, Mat4

class Game {
public:
  void init();
  void update(float dt);
  Mat4 view() const;

private:
  Vec3  camPos{0.f, 2.f, 5.f};
  float yaw{0.f};   // horizontal rotation in radians
  float pitch{0.f}; // vertical rotation in radians
  float logTimer{0.f};
  // --- Player physics ---
float m_velY = 0.0f;
bool  m_onGround = false;

// Tuning
float m_gravity = 9.81f;
float m_jumpSpeed = 6.5f;
float m_groundY = 0.0f;     // ground plane height

};
