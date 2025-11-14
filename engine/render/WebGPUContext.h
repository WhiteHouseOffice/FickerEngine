#pragma once

#include <cstdint>

namespace render {

// Placeholder type so Engine can compile without real WebGPU:
using WGPUTextureView = void*;

class WebGPUContext {
public:
  static WebGPUContext& Get();

  // Simple lifecycle – all stubbed for now
  void init();                       // called once from Engine
  void configure(int width, int height);
  bool isReady() const { return initialized; }

  WGPUTextureView acquireSwapchainView();
  void present(WGPUTextureView view);

  void resize(int width, int height);

private:
  bool initialized = false;
  int  width  = 0;
  int  height = 0;
};

} // namespace render
