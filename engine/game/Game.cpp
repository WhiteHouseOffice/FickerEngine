#include "game/Game.h"
#include "core/Input.h"
#include "math/MiniMath.h"

#include <cstdio>
#include <cmath>

namespace {

// Build a forward vector from yaw/pitch (in radians).
Vec3 computeForward(float yaw, float pitch) {
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  // Y-up, -Z forward when yaw = 0, pitch = 0
  return Vec3{
      sy * cp,   // x
      sp,        // y
     -cy * cp    // z
  };
}

Vec3 computeRight(const Vec3& forward) {
  const Vec3 worldUp{0.f, 1.f, 0.f};
  // right = forward x up
  Vec3 right = cross(forward, worldUp);
  float lenSq = right.x*right.x + right.y*right.y + right.z*right.z;
  if (lenSq > 0.0f) {
    float invLen = 1.0f / std::sqrt(lenSq);
    right.x *= invLen;
    right.y *= invLen;
    right.z *= invLen;
  }
  return right;
}

} // namespace

void Game::init() {
  camPos    = Vec3{0.f, 2.f, 5.f};
  yaw       = 0.0f;
  pitch     = 0.0f;
  logTimer  = 0.0f;
}

void Game::update(float dt) {
  // --- Mouse look ---
  float dx = 0.0f, dy = 0.0f;
  Input::getMouseDelta(dx, dy);

  const float mouseSensitivity = 0.0025f; // radians per pixel
  yaw   += dx * mouseSensitivity;
  pitch += -dy * mouseSensitivity; // invert so moving mouse up looks up

  // Clamp pitch to avoid flipping
  const float maxPitch = 1.5f; // ~86 degrees
  if (pitch >  maxPitch) pitch =  maxPitch;
  if (pitch < -maxPitch) pitch = -maxPitch;

  // --- Build camera basis vectors ---
  Vec3 forward = computeForward(yaw, pitch);
  Vec3 right   = computeRight(forward);
  const Vec3 up{0.f, 1.f, 0.f};

  // --- Keyboard movement ---
  Vec3 move{0.f, 0.f, 0.f};
  float speed = 3.0f;

  if (Input::isKeyDown(Input::KEY_W))     move += forward;
  if (Input::isKeyDown(Input::KEY_S))     move -= forward;
  if (Input::isKeyDown(Input::KEY_A))     move -= right;
  if (Input::isKeyDown(Input::KEY_D))     move += right;
  if (Input::isKeyDown(Input::KEY_SPACE)) move += up;
  if (Input::isKeyDown(Input::KEY_CTRL))  move -= up;

  if (Input::isKeyDown(Input::KEY_SHIFT)) speed *= 2.0f;

  // Normalize move vector so diagonal isn't faster
  float lenSq = move.x*move.x + move.y*move.y + move.z*move.z;
  if (lenSq > 0.0f) {
    float invLen = 1.0f / std::sqrt(lenSq);
    move.x *= invLen;
    move.y *= invLen;
    move.z *= invLen;

    camPos += move * (speed * dt);
  }

  // --- Debug camera log to console once per second ---
  logTimer += dt;
  if (logTimer >= 1.0f) {
    logTimer = 0.0f;
    std::printf("[camera] pos = (%.2f, %.2f, %.2f)\n",
                camPos.x, camPos.y, camPos.z);
  }
}

Mat4 Game::view() const {
  Vec3 forward = computeForward(yaw, pitch);
  const Vec3 up{0.f, 1.f, 0.f};
  const Vec3 target = camPos + forward;
  return lookAt(camPos, target, up);
}
