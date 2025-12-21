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

// Sphere vs plane y=0
static bool resolveSphereGround(Vec3& pos, Vec3& vel, float radius) {
  const float groundY = 0.0f;
  const float bottom = pos.y - radius;
  if (bottom < groundY) {
    pos.y += (groundY - bottom);
    if (vel.y < 0.0f) vel.y = 0.0f;
    return true;
  }
  return false;
}

// Sphere vs AABB
static bool resolveSphereAABB(Vec3& pos, Vec3& vel, float r, const AABB& b) {
  const float cx = clampf(pos.x, b.min.x, b.max.x);
  const float cy = clampf(pos.y, b.min.y, b.max.y);
  const float cz = clampf(pos.z, b.min.z, b.max.z);
  const Vec3 closest(cx, cy, cz);

  Vec3 d = pos - closest;
  const float dist2 = dot(d, d);
  if (dist2 > r * r) return false;

  float dist = std::sqrt(dist2);

  Vec3 n(0, 1, 0);
  if (dist > 1e-6f) {
    n = d * (1.0f / dist);
  } else {
    const float px = std::min(std::abs(pos.x - b.min.x), std::abs(b.max.x - pos.x));
    const float py = std::min(std::abs(pos.y - b.min.y), std::abs(b.max.y - pos.y));
    const float pz = std::min(std::abs(pos.z - b.min.z), std::abs(b.max.z - pos.z));
    if (py <= px && py <= pz) n = Vec3(0, (pos.y > (b.min.y + b.max.y) * 0.5f) ? 1.0f : -1.0f, 0);
    else if (px <= pz)       n = Vec3((pos.x > (b.min.x + b.max.x) * 0.5f) ? 1.0f : -1.0f, 0, 0);
    else                     n = Vec3(0, 0, (pos.z > (b.min.z + b.max.z) * 0.5f) ? 1.0f : -1.0f);
    dist = 0.0f;
  }

  const float penetration = r - dist;
  pos += n * penetration;

  const float vn = dot(vel, n);
  if (vn < 0.0f) vel -= n * vn;

  return true;
}

void Game::init() {
  m_platformCount = 0;
  m_platforms[m_platformCount++] = AABB{ Vec3(-2.0f, 0.5f, -2.0f), Vec3( 2.0f, 0.8f,  2.0f) };
  m_platforms[m_platformCount++] = AABB{ Vec3( 3.0f, 1.2f, -1.0f), Vec3( 4.5f, 1.5f,  1.0f) };
  m_platforms[m_platformCount++] = AABB{ Vec3(-4.0f, 0.2f,  3.0f), Vec3(-2.5f, 0.6f,  5.0f) };
}

void Game::update(float dt) {
  // Fly toggle (F edge-trigger)
  const bool fDown = Input::isKeyDown(Input::KEY_F);
  if (fDown && !m_prevFToggle) {
    m_flyMode = !m_flyMode;
    if (m_flyMode) {
      m_vel = Vec3(0,0,0);
      m_onGround = false;
    }
    printf("FlyMode: %s\n", m_flyMode ? "ON" : "OFF");
  }
  m_prevFToggle = fDown;

  // Mouse look
  float mdx = 0.0f, mdy = 0.0f;
  Input::getMouseDelta(mdx, mdy);
  m_yaw   += mdx * m_mouseSens;
  m_pitch += -mdy * m_mouseSens;

  const float pitchLimit = 1.55334f;
  m_pitch = clampf(m_pitch, -pitchLimit, pitchLimit);

  const Vec3 up(0,1,0);
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  const Vec3 right = normalize(cross(fwd, up));

  float speed = m_flyMode ? m_flySpeed : m_walkSpeed;
  if (Input::isKeyDown(Input::KEY_SHIFT)) speed *= m_sprintMul;

  Vec3 move(0,0,0);

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

    m_vel = Vec3(0,0,0);
    m_onGround = false;
  } else {
    // Walk: flatten forward so WASD never changes Y
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

    // Gravity + integrate
    m_vel.y -= m_gravity * dt;
    m_pos += m_vel * dt;

    bool groundedNow = false;
    groundedNow |= resolveSphereGround(m_pos, m_vel, m_radius);

    for (int i = 0; i < m_platformCount; ++i) {
      Vec3 before = m_pos;
      bool hit = resolveSphereAABB(m_pos, m_vel, m_radius, m_platforms[i]);
      if (hit && m_pos.y > before.y + 1e-4f) groundedNow = true;
    }

    m_onGround = groundedNow;

    // Optional: Ctrl = fall faster while airborne
    if (!m_onGround && Input::isKeyDown(Input::KEY_CTRL)) {
      m_vel.y -= (m_gravity * 1.5f) * dt;
    }
  }

  printf("pos: %.2f, %.2f, %.2f\n", m_pos.x, m_pos.y, m_pos.z);
}

Mat4 Game::view() const {
  const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
  const Vec3 target = m_pos + fwd;
  return lookAt(m_pos, target, Vec3(0,1,0));
}
