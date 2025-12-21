#include <cstdio>
#include <cmath>

#include "game/Game.h"
#include "core/Input.h"
#include "math/MiniMath.h"

static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Build forward/right from yaw/pitch
static Vec3 forwardFromYawPitch(float yaw, float pitch) {
  // yaw around Y, pitch around X
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);

  // Forward: (sinYaw*cosPitch, sinPitch, cosYaw*cosPitch)
  return normalize(Vec3(sy * cp, sp, cy * cp));
}

void Game::init() {
  // Nothing else needed; m_pos defaults are fine.
}

void Game::update(float dt) {
  // ---- Fly toggle on F press (edge-trigger) ----
  const bool fDown = Input::isKeyDown(Input::KEY_F);
  if (fDown && !m_prevFToggle) {
    m_flyMode = !m_flyMode;
    // When entering fly mode, kill vertical velocity so it feels clean.
    if (m_flyMode) {
      m_velY = 0.0f;
      m_onGround = false;
    }
    printf("FlyMode: %s\n", m_flyMode ? "ON" : "OFF");
  }
  m_prevFToggle = fDown;

  // ---- Mouse look ----
  float mdx = 0.0f, mdy = 0.0f;
  Input::getMouseDelta(mdx, mdy);

  m_yaw   += mdx * m_mouseSens;
  m_pitch += -mdy * m_mouseSens; // invert so mouse up looks up

  // Clamp pitch to avoid flipping
  const float pitchLimit = 1.55334f; // ~89 degrees
  m_pitch = clampf(m_pitch, -pitchLimit, pitchLimit);

  const Vec3 up = Vec3(0.0f, 1.0f, 0.0f);

  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  Vec3 right = normalize(cross(fwd, up));

  // ---- Speed ----
  float speed = m_flyMode ? m_flySpeed : m_walkSpeed;
  if (Input::isKeyDown(Input::KEY_SHIFT)) speed *= m_sprintMul;

  // ---- Movement intent ----
  Vec3 move(0.0f, 0.0f, 0.0f);

  if (m_flyMode) {
    // Fly mode: move in true camera forward/right; Space up; Ctrl down
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

    // No gravity in fly mode
    m_velY = 0.0f;
    m_onGround = false;
  } else {
    // Walk mode: project movement onto ground plane so W/A/S/D never changes Y
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

    // ---- Jump ----
    if (m_onGround && Input::isKeyDown(Input::KEY_SPACE)) {
      m_velY = m_jumpSpeed;
      m_onGround = false;
    }

    // ---- Gravity + ground collision ----
    m_velY -= m_gravity * dt;
    m_pos.y += m_velY * dt;

    if (m_pos.y <= m_groundY) {
      m_pos.y = m_groundY;
      m_velY = 0.0f;
      m_onGround = true;
    } else {
      m_onGround = false;
      // Optional: if you want Ctrl to “fall faster” while airborne (Minecraft-ish):
      if (Input::isKeyDown(Input::KEY_CTRL)) {
        m_velY -= (m_gravity * 1.5f) * dt;
      }
    }
  }

  // Debug print (matches what you saw earlier)
  printf("pos: %.2f, %.2f, %.2f\n", m_pos.x, m_pos.y, m_pos.z);
}

Mat4 Game::view() const {
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  const Vec3 target = m_pos + fwd;

  // MiniMath usually provides lookAt(). If yours doesn't, tell me and I’ll adapt.
  return lookAt(m_pos, target, Vec3(0.0f, 1.0f, 0.0f));
}
