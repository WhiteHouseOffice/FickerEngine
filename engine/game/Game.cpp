#include "game/Game.h"
#include "core/Input.h"
#include <cstdio>

void Game::init() {
  camPos     = { 0.0f, 2.0f, 5.0f };
  camForward = { 0.0f, 0.0f, -1.0f };
  moveSpeed  = 4.0f;
  logTimer   = 0.0f;
}

void Game::update(float dt) {
  // Flatten forward on XZ plane
  Vec3 fwd = camForward;
  fwd.y    = 0.0f;
  fwd      = normalize(fwd);

  Vec3 right = normalize(cross(fwd, Vec3{0.0f, 1.0f, 0.0f}));

  Vec3 delta{0.0f, 0.0f, 0.0f};
  float speed = moveSpeed * (Input::isKeyDown(Input::KEY_SHIFT) ? 2.0f : 1.0f);

  if (Input::isKeyDown(Input::KEY_W)) delta += fwd   * speed * dt;
  if (Input::isKeyDown(Input::KEY_S)) delta -= fwd   * speed * dt;
  if (Input::isKeyDown(Input::KEY_D)) delta += right * speed * dt;
  if (Input::isKeyDown(Input::KEY_A)) delta -= right * speed * dt;

  camPos += delta;

  // Periodically log camera coordinates
  logTimer += dt;
  if (logTimer >= 0.25f) {
    logTimer = 0.0f;
    std::printf("[Game] Camera position: (%.2f, %.2f, %.2f)\n",
                camPos.x, camPos.y, camPos.z);
  }
}

Mat4 Game::view() const {
  Vec3 up{0.0f, 1.0f, 0.0f};
  Vec3 target = camPos + camForward;
  return lookAt(camPos, target, up);
}
