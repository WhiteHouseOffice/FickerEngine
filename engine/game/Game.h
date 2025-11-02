#pragma once
#include "math/MiniMath.h"

// Minimal free-fly game controller with internal key handling for Web.
// No external Input.* dependency required.
class Game {
public:
    void Init();
    void Update(float dt);

    // Current view matrix (camera)
    const Mat4& View() const { return view_; }

private:
    // camera state
    float pos_[3]   { 0.0f, 1.6f, 6.0f };  // start a bit above the grid
    float yawDeg_   { -90.0f };            // -Z forward
    float pitchDeg_ {   0.0f };

    float baseSpeed_{ 3.0f };              // m/s
    float runMul_   { 2.0f };

    // cached view matrix
    Mat4  view_{};

    // input state (Web only; native stays false unless you wire input later)
    bool kW_=false, kA_=false, kS_=false, kD_=false;
    bool kQ_=false, kE_=false, kShift_=false;
    bool kUp_=false, kDown_=false, kLeft_=false, kRight_=false;

    // helpers
    void rebuildView_();
    void handleKeys_(float dt);
    static void normalize3_(float v[3]);
    static void cross3_(const float a[3], const float b[3], float out[3]);
};
