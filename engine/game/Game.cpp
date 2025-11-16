#include "game/Game.h"
#include "core/Input.h"
#include "math/MiniMath.h"

#include <cstdio>

void Game::init() {
  camPos    = Vec3{0.f, 2.f, 5.f};
  yaw       = 0.0f;
  logTimer  = 0.0f;
}

void Game::update(float dt) {
  Vec3 delta{0.f, 0.f, 0.f};
  const float speed = 3.0f;

  // Simple WASD movement in XZ plane
  if (Input::isKeyDown(Input::KEY_W)) delta.z -= speed * dt;
  if (Input::isKeyDown(Input::KEY_S)) delta.z += speed * dt;
  if (Input::isKeyDown(Input::KEY_A)) delta.x -= speed * dt;
  if (Input::isKeyDown(Input::KEY_D)) delta.x += speed * dt;

  camPos += delta;

  // Occasionally log camera position so we see life in the console
  logTimer += dt;
  if (logTimer >= 1.0f) {
    logTimer = 0.0f;
    std::printf("[camera] pos = (%.2f, %.2f, %.2f)\n",
                camPos.x, camPos.y, camPos.z);
  }
}

Mat4 Game::view() const {
  const Vec3 forward{0.f, 0.f, -1.f};
  const Vec3 up{0.f, 1.f, 0.f};
  const Vec3 target = camPos + forward;
  return lookAt(camPos, target, up);
}
