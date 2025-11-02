#include "game/Game.h"
#include "math/MiniMath.h"
#include "core/Input.h"    // your existing input system
#include <cstdio>
#include <algorithm>

void Game::Init() {
    camPos = { 0.f, 2.f, 5.f };
    yaw = 0.0f;
}

void Game::Update(float dt) {
    // Basic WASD on XZ plane; hold SHIFT to go faster
    float speed = moveSpeed * (Input::isKeyDown(Input::KEY_SHIFT) ? 2.0f : 1.0f);

    // Forward vector from yaw (on XZ)
    float cy = cosf(yaw), sy = sinf(yaw);
    Vec3 fwd {  sy, 0.f, -cy };
    Vec3 right{  cy, 0.f,  sy };

    Vec3 delta {0,0,0};
    if (Input::isKeyDown(Input::KEY_W)) delta += fwd * speed * dt;
    if (Input::isKeyDown(Input::KEY_S)) delta -= fwd * speed * dt;
    if (Input::isKeyDown(Input::KEY_D)) delta += right * speed * dt;
    if (Input::isKeyDown(Input::KEY_A)) delta -= right * speed * dt;

    // Optional: space/ctrl up/down
    if (Input::isKeyDown(Input::KEY_SPACE)) delta.y += speed * dt;
    if (Input::isKeyDown(Input::KEY_CTRL))  delta.y -= speed * dt;

    camPos += delta;

    // Periodic console log of coordinates (4×/sec)
    logTimer += dt;
    if (logTimer >= 0.25f) {
        logTimer = 0.f;
        std::printf("[pos] x=%.2f y=%.2f z=%.2f\n", camPos.x, camPos.y, camPos.z);
        std::fflush(stdout);
    }
}

Mat4 Game::View() const {
    // Build a look-at with yaw only (keeps horizon level)
    float cy = cosf(yaw), sy = sinf(yaw);
    Vec3 fwd { sy, 0.f, -cy };
    Vec3 target = camPos + fwd;
    return lookAt(camPos, target, Vec3{0,1,0});
}
