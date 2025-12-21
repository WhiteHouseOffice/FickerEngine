#pragma once

#include "math/MiniMath.h"

// World collider type (axis-aligned boxes)
struct AABB {
  Vec3 min;
  Vec3 max;
};

class Game {
public:
  void init();
  void update(float dt);

  // Used by Engine.cpp
  Mat4 view() const;

private:
  // Player "feet" position (ground contact is y=0)
  Vec3  m_pos   = Vec3(0.0f, 0.0f, 5.0f);

  // Camera orientation
  float m_yaw   = 3.14159265f;
  float m_pitch = 0.0f;

  // Camera eye height (feels like human height now)
  float m_eyeHeight = 1.0f;

  // Player collider: sphere centered above feet by radius
  float m_radius = 0.35f;
  Vec3  m_vel    = Vec3(0.0f, 0.0f, 0.0f); // only Y used in walk mode
  bool  m_onGround = false;

  // Modes
  bool  m_flyMode = false;
  bool  m_prevFToggle = false;

  // Tunables
  float m_gravity   = 18.0f;
  float m_jumpSpeed = 7.0f;

  float m_walkSpeed = 6.0f;
  float m_flySpeed  = 8.0f;
  float m_sprintMul = 2.0f;

  // Mouse tuning (stable)
  float m_mouseSens = 0.0008f;

  // Simple static world: platforms you already see drawn
  static constexpr int kMaxPlatforms = 8;
  AABB m_platforms[kMaxPlatforms];
  int  m_platformCount = 0;
};
