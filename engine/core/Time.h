#pragma once

namespace Time {
    // Call once at startup
    void init();
    // Call once per frame
    void update();
    // Seconds since last update()
    float deltaTime();
}
