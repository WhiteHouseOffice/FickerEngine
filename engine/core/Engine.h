#pragma once
#include <memory>

class Engine {
public:
    Engine();
    ~Engine();

    // --- Main lifecycle methods ---
    void init();       // initialize systems and WebGPU
    void update();     // per-frame logic
    void render();     // per-frame rendering
    void shutdown();   // cleanup

private:
    // (future: add Scene*, Game*, etc.)
};
