#include "game/Game.h"
#include "core/Input.h"

#include <cstdio>
#include <cmath>

void Game::init() {
  camPos   = Vec3{0.f, 2.f, 5.f};
  yaw      = 0.f;
  pitch    = 0.f;
  logTimer = 0.f;
}

void Game::update(float dt) {
  // --- movement vectors (XZ plane, yaw-based) ---
  Vec3 fwd{
    std::sin(yaw),   // x
    0.f,             // y
    -std::cos(yaw)   // z
  };
  Vec3 right{
    std::cos(yaw),
    0.f,
    std::sin(yaw)
  };

  float speed = moveSpeed * (Input::isKeyDown(Input::KEY_SHIFT) ? 2.0f : 1.0f);
  Vec3 delta{0.f, 0.f, 0.f};

  if (Input::isKeyDown(Input::KEY_W)) delta += fwd   * speed * dt;
  if (Input::isKeyDown(Input::KEY_S)) delta -= fwd   * speed * dt;
  if (Input::isKeyDown(Input::KEY_D)) delta += right * speed * dt;
  if (Input::isKeyDown(Input::KEY_A)) delta -= right * speed * dt;

  camPos += delta;

  // yaw rotation with Q/E
  if (Input::isKeyDown(Input::KEY_Q)) yaw -= 1.0f * dt;
  if (Input::isKeyDown(Input::KEY_E)) yaw += 1.0f * dt;

  // log camera position every 0.25s
  logTimer += dt;
  if (logTimer >= 0.25f) {
    logTimer = 0.f;
    std::printf("[Game] Camera pos: x=%.2f y=%.2f z=%.2f (yaw=%.2f)\n",
                camPos.x, camPos.y, camPos.z, yaw);
  }
}

Mat4 Game::view() const {
  const float degToRad = 3.1415926535f / 180.0f;
  float pitchClamped = pitch;
  if (pitchClamped >  89.0f * degToRad) pitchClamped =  89.0f * degToRad;
  if (pitchClamped < -89.0f * degToRad) pitchClamped = -89.0f * degToRad;

  float cp = std::cos(pitchClamped);
  float sp = std::sin(pitchClamped);
  float cy = std::cos(yaw);
  float sy = std::sin(yaw);

  Vec3 dir{
    cp * sy,
    sp,
    -cp * cy
  };

  Vec3 target = camPos + dir;
  Vec3 up{0.f, 1.f, 0.f};

  return lookAt(camPos, target, up);
}

Mat4 Game::proj(float aspect) const {
  const float degToRad = 3.1415926535f / 180.0f;
  float fov = 60.0f * degToRad;
  return perspective(fov, aspect, 0.1f, 500.0f);
}
