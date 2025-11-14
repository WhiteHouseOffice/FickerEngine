#include "game/Game.h"

#include "core/Input.h"
#include <cmath>

void Game::init() {
  cameraPos = Vec3(0.0f, 1.5f, 5.0f);
  yaw       = 0.0f;
}

void Game::update(float dt) {
  float speed = moveSpeed;

  // Optional "sprint" if KEY_SHIFT exists in your Input.h
  if (Input::isKeyDown(Input::KEY_SHIFT)) {
    speed *= 2.0f;
  }

  Vec3 delta(0.0f, 0.0f, 0.0f);

  // Forward/right based on yaw (XZ-plane)
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  Vec3 forward(sy, 0.0f, -cy);
  Vec3 right  (cy, 0.0f,  sy);

  // WASD movement only (no SPACE/CTRL/arrow keys anymore)
  if (Input::isKeyDown(Input::KEY_W)) delta += forward * (speed * dt);
  if (Input::isKeyDown(Input::KEY_S)) delta -= forward * (speed * dt);
  if (Input::isKeyDown(Input::KEY_A)) delta -= right   * (speed * dt);
  if (Input::isKeyDown(Input::KEY_D)) delta += right   * (speed * dt);

  cameraPos += delta;
}

Mat4 Game::view() const {
  // Placeholder until we hook into a real lookAt in MiniMath
  Mat4 m{};    // zero-init; replace with proper view matrix later
  return m;
}
