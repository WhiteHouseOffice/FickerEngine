#include "Game.h"
#include <cmath>

#if defined(FE_WEB)
  #include <emscripten/html5.h>
#endif

static inline float deg2rad(float d){ return d * 3.1415926535f / 180.0f; }

void Game::Init() {
    // identity to start
    matIdentity(view_);

#if defined(FE_WEB)
    // Simple key listeners to avoid extra platform code or folders.
    auto keyHandler = [](int eventType, const EmscriptenKeyboardEvent* e, void* user) -> EM_BOOL {
        Game* g = reinterpret_cast<Game*>(user);
        const bool down = (eventType == EMSCRIPTEN_EVENT_KEYDOWN);

        // Use DOM "key" string; examples: "w","a","Shift","ArrowUp"
        const char* k = e->key;
        auto eq = [&](const char* s){ return std::strcmp(k, s) == 0; };

        if (eq("w")) g->kW_ = down;
        else if (eq("a")) g->kA_ = down;
        else if (eq("s")) g->kS_ = down;
        else if (eq("d")) g->kD_ = down;
        else if (eq("q")) g->kQ_ = down;
        else if (eq("e")) g->kE_ = down;
        else if (eq("Shift")) g->kShift_ = down;
        else if (eq("ArrowUp")) g->kUp_ = down;
        else if (eq("ArrowDown")) g->kDown_ = down;
        else if (eq("ArrowLeft")) g->kLeft_ = down;
        else if (eq("ArrowRight")) g->kRight_ = down;

        // prevent page scroll on arrows/space etc.
        return EM_TRUE;
    };

    emscripten_set_keydown_callback(EMSCRIPTEN_EVENT_TARGET_DOCUMENT, this, true, keyHandler);
    emscripten_set_keyup_callback   (EMSCRIPTEN_EVENT_TARGET_DOCUMENT, this, true, keyHandler);
#endif
}

void Game::Update(float dt) {
    // arrows rotate camera (yaw/pitch)
    const float rotSpeed = 90.0f; // deg/sec
    if (kLeft_)  yawDeg_   -= rotSpeed * dt;
    if (kRight_) yawDeg_   += rotSpeed * dt;
    if (kUp_)    pitchDeg_ += rotSpeed * dt;
    if (kDown_)  pitchDeg_ -= rotSpeed * dt;

    // clamp pitch to avoid gimbal flip
    if (pitchDeg_ >  89.0f) pitchDeg_ =  89.0f;
    if (pitchDeg_ < -89.0f) pitchDeg_ = -89.0f;

    handleKeys_(dt);
    rebuildView_();
}

void Game::handleKeys_(float dt) {
    // Build basis from yaw/pitch
    const float yaw   = deg2rad(yawDeg_);
    const float pitch = deg2rad(pitchDeg_);

    float fwd[3] = { std::cos(yaw)*std::cos(pitch),
                     std::sin(pitch),
                     std::sin(yaw)*std::cos(pitch) };
    normalize3_(fwd);

    const float up[3] = {0.0f, 1.0f, 0.0f};
    float right[3]; cross3_(fwd, up, right); normalize3_(right);

    float vel = baseSpeed_ * (kShift_ ? runMul_ : 1.0f);
    float move[3] = {0,0,0};

    auto addScaled = [&](const float v[3], float s){
        move[0] += v[0]*s; move[1] += v[1]*s; move[2] += v[2]*s;
    };

    if (kW_) addScaled(fwd, +vel);
    if (kS_) addScaled(fwd, -vel);
    if (kD_) addScaled(right, +vel);
    if (kA_) addScaled(right, -vel);
    if (kE_) move[1] += vel;
    if (kQ_) move[1] -= vel;

    // integrate
    pos_[0] += move[0]*dt;
    pos_[1] += move[1]*dt;
    pos_[2] += move[2]*dt;
}

void Game::rebuildView_() {
    const float yaw   = deg2rad(yawDeg_);
    const float pitch = deg2rad(pitchDeg_);

    float fwd[3] = { std::cos(yaw)*std::cos(pitch),
                     std::sin(pitch),
                     std::sin(yaw)*std::cos(pitch) };
    float tgt[3] = { pos_[0] + fwd[0], pos_[1] + fwd[1], pos_[2] + fwd[2] };
    const float up[3] = {0.0f, 1.0f, 0.0f};

    view_ = lookAt(pos_, tgt, up);
}

void Game::normalize3_(float v[3]){
    float L = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if (L > 1e-6f){ v[0]/=L; v[1]/=L; v[2]/=L; }
}

void Game::cross3_(const float a[3], const float b[3], float out[3]){
    out[0] = a[1]*b[2]-a[2]*b[1];
    out[1] = a[2]*b[0]-a[0]*b[2];
    out[2] = a[0]*b[1]-a[1]*b[0];
}
