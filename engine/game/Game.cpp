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
  m_onGround = true;
  m_velY = 0.0f;

  // Start standing on ground
  m_pos = Vec3(0.0f, 0.0f, 5.0f);
  m_yaw = 3.14159265f;
  m_pitch = 0.0f;

  printf("[Game] init (WALK)\n");
}

void Game::update(float dt) {
  // Toggle fly mode (F edge-trigger)
  const bool fDown = Input::isKeyDown(Input::KEY_F);
  if (fDown && !m_prevFToggle) {
    m_flyMode = !m_flyMode;
    m_velY = 0.0f;
    m_onGround = !m_flyMode;
    printf("[mode] %s\n", m_flyMode ? "FLY" : "WALK");
  }
  m_prevFToggle = fDown;

  // --- Mouse look (stabilized) ---
  float mdx = 0.0f, mdy = 0.0f;
  Input::getMouseDelta(mdx, mdy);

  // Clamp crazy deltas so one bad frame doesn't pin pitch
  const float clampDelta = 50.0f;
  if (mdx >  clampDelta) mdx =  clampDelta;
  if (mdx < -clampDelta) mdx = -clampDelta;
  if (mdy >  clampDelta) mdy =  clampDelta;
  if (mdy < -clampDelta) mdy = -clampDelta;

  m_yaw += mdx * m_mouseSens;

  // IMPORTANT:
  // This sign fixes your "only look down" issue on your current setup.
  // If you want inverted later, we can expose a flag.
  m_pitch += (mdy) * m_mouseSens;

  const float pitchLimit = 1.55334f; // ~89 degrees
  m_pitch = clampf(m_pitch, -pitchLimit, pitchLimit);

  // Basis
  const Vec3 up(0,1,0);
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  const Vec3 right = normalize(cross(fwd, up));

  float speed = m_flyMode ? m_flySpeed : m_walkSpeed;
  if (Input::isKeyDown(Input::KEY_SHIFT)) speed *= m_sprintMul;

  Vec3 move(0,0,0);

  if (m_flyMode) {
    // Fly: full camera direction, plus vertical controls
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

    // No ground clamping in fly mode (as you wanted)
    m_velY = 0.0f;
    m_onGround = false;
  } else {
    // Walk: project movement to XZ plane (prevents W affecting Y)
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
      m_velY = m_jumpSpeed;
      m_onGround = false;
    }

    // Gravity + integrate
    m_velY -= m_gravity * dt;
    m_pos.y += m_velY * dt;

    // Ground plane at y=0 for FEET
    if (m_pos.y <= 0.0f) {
      m_pos.y = 0.0f;
      m_velY = 0.0f;
      m_onGround = true;
    } else {
      m_onGround = false;

      // Optional: Ctrl makes you fall faster (Minecraft-ish)
      if (Input::isKeyDown(Input::KEY_CTRL)) {
        m_velY -= (m_gravity * 1.5f) * dt;
      }
    }
  }

  // Debug: show feet position + mode
  printf("pos(feet): %.2f, %.2f, %.2f | %s\n", m_pos.x, m_pos.y, m_pos.z, m_flyMode ? "FLY" : "WALK");
}

Mat4 Game::view() const {
  // Camera eye is above feet
  const Vec3 eye = m_pos + Vec3(0.0f, m_eyeHeight, 0.0f);
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  return lookAt(eye, eye + fwd, Vec3(0,1,0));
}
