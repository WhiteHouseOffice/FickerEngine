#include <cstdio>
#include <cmath>

#include "game/Game.h"
#include "core/Input.h"
#include "math/MiniMath.h"

static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static Vec3 forwardFromYawPitch(float yaw, float pitch) {
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  return normalize(Vec3(sy * cp, sp, cy * cp));
}

void Game::init() {
  m_flyMode = false;
  m_prevFToggle = false;
  m_onGround = false;
  m_vel = Vec3(0,0,0);

  printf("[Game] init\n");
}

void Game::update(float dt) {
  /* ============================================================
     MODE TOGGLE (F)
     ============================================================ */
  const bool fDown = Input::isKeyDown(Input::KEY_F);
  if (fDown && !m_prevFToggle) {
    m_flyMode = !m_flyMode;
    m_vel = Vec3(0,0,0);
    m_onGround = false;
    printf("[mode] %s\n", m_flyMode ? "FLY" : "WALK");
  }
  m_prevFToggle = fDown;

  /* ============================================================
     MOUSE LOOK  (THIS IS THE FULL BLOCK YOU ASKED FOR)
     ============================================================ */
  float mdx = 0.0f, mdy = 0.0f;
  Input::getMouseDelta(mdx, mdy);

  // Clamp insane deltas (prevents pitch lock)
  if (mdx > 200.0f) mdx = 200.0f;
  if (mdx < -200.0f) mdx = -200.0f;
  if (mdy > 200.0f) mdy = 200.0f;
  if (mdy < -200.0f) mdy = -200.0f;

  // Mouse up = look up (correct for GLFW)
  m_yaw   += mdx * m_mouseSens;
  m_pitch += (-mdy) * m_mouseSens;

  const float pitchLimit = 1.55334f; // ~89 degrees
  m_pitch = clampf(m_pitch, -pitchLimit, pitchLimit);

  /* ============================================================
     CAMERA BASIS
     ============================================================ */
  const Vec3 up(0,1,0);
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  const Vec3 right = normalize(cross(fwd, up));

  float speed = m_flyMode ? m_flySpeed : m_walkSpeed;
  if (Input::isKeyDown(Input::KEY_SHIFT)) speed *= m_sprintMul;

  Vec3 move(0,0,0);

  /* ============================================================
     FLY MODE (CREATIVE)
     ============================================================ */
  if (m_flyMode) {
    if (Input::isKeyDown(Input::KEY_W)) move += fwd;
    if (Input::isKeyDown(Input::KEY_S)) move -= fwd;
    if (Input::isKeyDown(Input::KEY_A)) move -= right;
    if (Input::isKeyDown(Input::KEY_D)) move += right;
    if (Input::isKeyDown(Input::KEY_SPACE)) move += up;
    if (Input::isKeyDown(Input::KEY_CTRL))  move -= up;

    if (length(move) > 0.0001f) {
      move = normalize(move);
      m_pos += move * (speed * dt);
    }

    // No gravity
    m_vel = Vec3(0,0,0);
    m_onGround = false;
  }

  /* ============================================================
     WALK MODE (GROUND + GRAVITY)
     ============================================================ */
  else {
    // Project movement onto ground plane (prevents W lifting you)
    Vec3 fwdFlat(fwd.x, 0.0f, fwd.z);
    if (length(fwdFlat) > 0.0001f) fwdFlat = normalize(fwdFlat);
    Vec3 rightFlat = normalize(cross(fwdFlat, up));

    if (Input::isKeyDown(Input::KEY_W)) move += fwdFlat;
    if (Input::isKeyDown(Input::KEY_S)) move -= fwdFlat;
    if (Input::isKeyDown(Input::KEY_A)) move -= rightFlat;
    if (Input::isKeyDown(Input::KEY_D)) move += rightFlat;

    if (length(move) > 0.0001f) {
      move = normalize(move);
      m_pos += move * (speed * dt);
    }

    // Jump
    if (m_onGround && Input::isKeyDown(Input::KEY_SPACE)) {
      m_vel.y = m_jumpSpeed;
      m_onGround = false;
    }

    // Gravity
    m_vel.y -= m_gravity * dt;
    m_pos.y += m_vel.y * dt;

    // Ground plane y = 0
    if (m_pos.y <= 0.0f) {
      m_pos.y = 0.0f;
      m_vel.y = 0.0f;
      m_onGround = true;
    } else {
      m_onGround = false;
    }
  }

  // Debug
  printf("pos: %.2f, %.2f, %.2f\n", m_pos.x, m_pos.y, m_pos.z);
}

Mat4 Game::view() const {
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  return lookAt(m_pos, m_pos + fwd, Vec3(0,1,0));
}
