#include "game/Game.h"

#include "core/Input.h"
#include <cmath>

void Game::init() {
  cameraPos = Vec3(0.0f, 1.5f, 5.0f);
  yaw       = 0.0f;
}

void Game::update(float dt) {
  float speed = moveSpeed;

  // optional: sprint with SHIFT if it exists
  if (Input::isKeyDown(Input::KEY_SHIFT)) {
    speed *= 2.0f;
  }

  Vec3 delta(0.0f, 0.0f, 0.0f);

  // Forward/right in XZ plane based on yaw
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  Vec3 forward(sy, 0.0f, -cy);
  Vec3 right  (cy, 0.0f,  sy);

  // WASD movement
  if (Input::isKeyDown(Input::KEY_W)) delta += forward * (speed * dt);
  if (Input::isKeyDown(Input::KEY_S)) delta -= forward * (speed * dt);
  if (Input::isKeyDown(Input::KEY_A)) delta -= right   * (speed * dt);
  if (Input::isKeyDown(Input::KEY_D)) delta += right   * (speed * dt);

  // Horizontal look with arrow keys (if they exist)
  //if (Input::isKeyDown(Input::KEY_LEFT))  yaw -= 1.5f * dt;
  //if (Input::isKeyDown(Input::KEY_RIGHT)) yaw += 1.5f * dt;

  cameraPos += delta;
}

Mat4 Game::view() const {
  // For now just return a default matrix (we’ll hook into MiniMath later)
  Mat4 view{};
  return view;
}
