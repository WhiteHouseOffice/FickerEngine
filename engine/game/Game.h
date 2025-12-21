#pragma once

#include "math/MiniMath.h"

class Game {
public:
  void init();
  void update(float dt);

  // Used by Engine.cpp
  Mat4 view() const;

private:
  // Player "feet" position (NOT the camera eye)
  Vec3  m_pos   = Vec3(0.0f, 0.0f, 5.0f);

  // Camera orientation
  float m_yaw   = 3.14159265f; // facing toward -Z-ish
  float m_pitch = 0.0f;

  // Eye height above feet so you're not inside the ground
  float m_eyeHeight = 1.6f;

  // Physics
  float m_velY = 0.0f;
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

  // MUCH lower sensitivity (raw mouse motion is strong)
  float m_mouseSens = 0.0010f;
};
