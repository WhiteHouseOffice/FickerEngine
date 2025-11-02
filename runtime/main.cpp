#include "core/Engine.h"
#include <thread>
#include <chrono>
int main(){
    Engine::instance().init();
    for (int i=0;i<300;i++){ // ~5 seconds at 60fps
        Engine::instance().stepOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
}
