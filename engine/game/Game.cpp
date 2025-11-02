#include "game/Game.h"
#include "core/Input.h"
#include <cstdio>
#include <cmath>

#if !__has_include("math/MiniMath.h")
static inline Vec3 normalize(const Vec3& v){ float L=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); if(L==0) return {0,0,0}; return {v.x/L,v.y/L,v.z/L}; }
static inline Vec3 cross(const Vec3& a,const Vec3& b){ return { a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x }; }
static Mat4 lookAt(const Vec3& eye, const Vec3& at, const Vec3& up) {
    Vec3 f = normalize({at.x-eye.x, at.y-eye.y, at.z-eye.z});
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 M{};
    M.m[0]= s.x; M.m[4]= s.y; M.m[8] = s.z; M.m[12]= -(s.x*eye.x + s.y*eye.y + s.z*eye.z);
    M.m[1]= u.x; M.m[5]= u.y; M.m[9] = u.z; M.m[13]= -(u.x*eye.x + u.y*eye.y + u.z*eye.z);
    M.m[2]= -f.x;M.m[6]= -f.y;M.m[10]= -f.z;M.m[14]=  (f.x*eye.x + f.y*eye.y + f.z*eye.z);
    M.m[3]= 0;   M.m[7]= 0;   M.m[11]= 0;   M.m[15]= 1;
    return M;
}
#endif

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
