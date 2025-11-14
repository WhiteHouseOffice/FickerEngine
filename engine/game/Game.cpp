#include "game/Game.h"
#include "core/Input.h"
#include "math/MiniMath.h"

#include <cmath>
#include <cstdio>

void Game::init() {
  camPos    = Vec3{0.f, 2.f, 5.f};
  yaw       = 0.0f;
  moveSpeed = 2.0f;
  logTimer  = 0.0f;
}

void Game::update(float dt) {
  float speed = moveSpeed * (Input::isKeyDown(Input::KEY_SHIFT) ? 2.0f : 1.0f);

  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  const Vec3 fwd   { sy, 0.f, -cy };
  const Vec3 right { cy, 0.f,  sy };

  Vec3 delta{0.f, 0.f, 0.f};

  if (Input::isKeyDown(Input::KEY_W))     delta += fwd   * speed * dt;
  if (Input::isKeyDown(Input::KEY_S))     delta -= fwd   * speed * dt;
  if (Input::isKeyDown(Input::KEY_D))     delta += right * speed * dt;
  if (Input::isKeyDown(Input::KEY_A))     delta -= right * speed * dt;
  if (Input::isKeyDown(Input::KEY_SPACE)) delta.y += speed * dt;
  if (Input::isKeyDown(Input::KEY_CTRL))  delta.y -= speed * dt;

  camPos += delta;

  // Periodically log the camera position so we can see movement in the Actions log.
  logTimer += dt;
  if (logTimer >= 0.25f) {
    logTimer = 0.f;
    std::printf("[camera] pos = (%.2f, %.2f, %.2f)\n", camPos.x, camPos.y, camPos.z);
    std::fflush(stdout);
  }
}

Mat4 Game::view() const {
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  const Vec3 fwd    { sy, 0.f, -cy };
  const Vec3 target = camPos + fwd;
  const Vec3 up     { 0.f, 1.f, 0.f };

  return lookAt(camPos, target, up);
}
