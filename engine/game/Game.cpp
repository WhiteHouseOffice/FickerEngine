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

/* ====================== COLLISION HELPERS ====================== */

static bool resolveSphereGround(Vec3& center, Vec3& vel, float radius) {
    const float bottom = center.y - radius;
    if (bottom < 0.0f) {
        center.y -= bottom;
        if (vel.y < 0.0f) vel.y = 0.0f;
        return true;
    }
    return false;
}

static bool resolveSphereAABB(Vec3& center, Vec3& vel, float r, const AABB& b) {
    const float cx = clampf(center.x, b.min.x, b.max.x);
    const float cy = clampf(center.y, b.min.y, b.max.y);
    const float cz = clampf(center.z, b.min.z, b.max.z);
    Vec3 closest(cx, cy, cz);

    Vec3 d = center - closest;
    float dist2 = dot(d, d);
    if (dist2 > r * r) return false;

    float dist = std::sqrt(dist2);
    Vec3 n(0, 1, 0);

    if (dist > 1e-6f) {
        n = d * (1.0f / dist);
    }

    float penetration = r - dist;
    center += n * penetration;

    float vn = dot(vel, n);
    if (vn < 0.0f) vel -= n * vn;

    return true;
}

/* ====================== GAME ====================== */

void Game::init() {
    m_pos = Vec3(0.0f, 0.0f, 5.0f);
    m_vel = Vec3(0.0f, 0.0f, 0.0f);
    m_onGround = true;

    m_yaw = 3.14159265f;
    m_pitch = 0.0f;

    m_flyMode = false;
    m_prevFToggle = false;

    /* Platforms (must match Scene debug draw) */
    m_platformCount = 0;
    m_platforms[m_platformCount++] = AABB{
        Vec3(-2.0f, 0.5f, -2.0f),
        Vec3( 2.0f, 0.8f,  2.0f)
    };
    m_platforms[m_platformCount++] = AABB{
        Vec3( 3.0f, 1.2f, -1.0f),
        Vec3( 4.5f, 1.5f,  1.0f)
    };
    m_platforms[m_platformCount++] = AABB{
        Vec3(-4.0f, 0.2f,  3.0f),
        Vec3(-2.5f, 0.6f,  5.0f)
    };

    printf("[Game] init WALK\n");
}

void Game::update(float dt) {
    /* ---------- MODE TOGGLE ---------- */
    bool fDown = Input::isKeyDown(Input::KEY_F);
    if (fDown && !m_prevFToggle) {
        m_flyMode = !m_flyMode;
        m_vel = Vec3(0,0,0);
        m_onGround = !m_flyMode;
        printf("[mode] %s\n", m_flyMode ? "FLY" : "WALK");
    }
    m_prevFToggle = fDown;

    /* ---------- MOUSE LOOK ---------- */
 // ---- mouse look (RAW mouse deltas -> yaw/pitch) ----
float mdx = 0.0f, mdy = 0.0f;
Input::getMouseDelta(mdx, mdy);

// Sensitivity (ONLY tuning knob)
const float sens = 0.0025f; // radians per pixel

// Inversion (change ONLY here)
const bool invertX = false;
const bool invertY = true;

if (invertX) mdx = -mdx;
if (invertY) mdy = -mdy;

// Apply rotation
m_yaw   -= mdx * sens;
m_pitch += mdy * sens;

// --- yaw: infinite, wrapped ---
const float twoPi = 6.28318530718f;
if (m_yaw >  twoPi) m_yaw -= twoPi;
if (m_yaw < -twoPi) m_yaw += twoPi;

// --- pitch: clamped ---
const float maxPitch = 1.55334306f; // ~89°
if (m_pitch >  maxPitch)  m_pitch =  maxPitch;
if (m_pitch < -maxPitch)  m_pitch = -maxPitch;

    /* ---------- BASIS ---------- */
    const Vec3 up(0,1,0);
    const Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
    const Vec3 right = normalize(cross(fwd, up));

    float speed = m_flyMode ? m_flySpeed : m_walkSpeed;
    if (Input::isKeyDown(Input::KEY_SHIFT)) speed *= m_sprintMul;

    Vec3 move(0,0,0);

    /* ---------- FLY MODE ---------- */
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
        return;
    }

    /* ---------- WALK MODE ---------- */
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

    Vec3 center = m_pos + Vec3(0.0f, m_radius, 0.0f);

    if (m_onGround && Input::isKeyDown(Input::KEY_SPACE)) {
        m_vel.y = m_jumpSpeed;
        m_onGround = false;
    }

    m_vel.y -= m_gravity * dt;
    center.y += m_vel.y * dt;

    bool groundedNow = false;
    groundedNow |= resolveSphereGround(center, m_vel, m_radius);

    for (int i = 0; i < m_platformCount; ++i) {
        Vec3 before = center;
        if (resolveSphereAABB(center, m_vel, m_radius, m_platforms[i]) &&
            center.y > before.y + 1e-4f) {
            groundedNow = true;
        }
    }

    m_pos = center - Vec3(0.0f, m_radius, 0.0f);
    m_onGround = groundedNow;
}

Mat4 Game::view() const {
    Vec3 eye = m_pos + Vec3(0.0f, m_eyeHeight, 0.0f);
    Vec3 fwd = forwardFromYawPitch(m_yaw, m_pitch);
    return lookAt(eye, eye + fwd, Vec3(0,1,0));
}


void Game::applySceneCorrection(const Vec3& correctedCenter, const Vec3& correctedVelocity, bool groundedFromScene) {
    // Convert sphere center back to feet position
    m_pos = correctedCenter - Vec3(0.0f, m_radius, 0.0f);

    // Keep velocity consistent with collision response
    m_vel = correctedVelocity;

    if (groundedFromScene) {
        m_onGround = true;
        if (m_vel.y < 0.0f) m_vel.y = 0.0f;
    }
}
