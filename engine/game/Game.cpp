#include "game/Game.h"

// Prefer the authoritative Input in engine/core/, but fall back if needed.
#if __has_include("core/Input.h")
  #include "core/Input.h"
#elif __has_include("input/Input.h")
  #include "input/Input.h"
#else
  // Minimal stub so it still compiles if Input isn’t present in CI.
  namespace Input {
    enum Key { KEY_W, KEY_A, KEY_S, KEY_D, KEY_SHIFT, KEY_SPACE, KEY_CTRL };
    inline bool isKeyDown(Key){ return false; }
  }
#endif

#include <cstdio>
#include <cmath>

void Game::Init() {
    camPos = { 0.f, 2.f, 5.f };
    yaw = 0.0f;
}

void Game::Update(float dt) {
    float speed = moveSpeed * (Input::isKeyDown(Input::KEY_SHIFT) ? 2.0f : 1.0f);

    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const Vec3 fwd   {  sy, 0.f, -cy };
    const Vec3 right {  cy, 0.f,  sy };

    Vec3 delta {0,0,0};
    if (Input::isKeyDown(Input::KEY_W)) delta += fwd   * speed * dt;
    if (Input::isKeyDown(Input::KEY_S)) delta -= fwd   * speed * dt;
    if (Input::isKeyDown(Input::KEY_D)) delta += right * speed * dt;
    if (Input::isKeyDown(Input::KEY_A)) delta -= right * speed * dt;

    if (Input::isKeyDown(Input::KEY_SPACE)) delta.y += speed * dt;
    if (Input::isKeyDown(Input::KEY_CTRL))  delta.y -= speed * dt;

    camPos += delta;

    logTimer += dt;
    if (logTimer >= 0.25f) {
        logTimer = 0.f;
        std::printf("[pos] x=%.2f y=%.2f z=%.2f\n", camPos.x, camPos.y, camPos.z);
        std::fflush(stdout);
    }
}

Mat4 Game::View() const {
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const Vec3 fwd { sy, 0.f, -cy };
    const Vec3 target = camPos + fwd;
    return lookAt(camPos, target, Vec3{0,1,0});
}
