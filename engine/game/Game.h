#pragma once

#include "math/MiniMath.h"

class Game {
public:
  void init();
  void update(float dt);

  // Used by Engine.cpp
  Mat4 view() const;

private:
  // Camera/player state
  Vec3  m_pos   = Vec3(0.0f, 2.0f, 5.0f);
  float m_yaw   = 3.14159265f; // looking toward -Z-ish initially
  float m_pitch = 0.0f;

  // Physics
  float m_velY = 0.0f;
  bool  m_onGround = false;

  // Modes
  bool  m_flyMode = false;
  bool  m_prevFToggle = false;

  // Tunables
  float m_groundY   = 0.0f;
  float m_gravity   = 18.0f;
  float m_jumpSpeed = 7.0f;

  float m_walkSpeed = 6.0f;
  float m_flySpeed  = 8.0f;
  float m_sprintMul = 2.0f;

  float m_mouseSens = 0.0025f;
};
