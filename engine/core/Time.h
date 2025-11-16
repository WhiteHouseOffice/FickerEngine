#pragma once

namespace Time {

void init();
void update();

float deltaTime(); // seconds since last frame
float time();      // total time since start

} // namespace Time
