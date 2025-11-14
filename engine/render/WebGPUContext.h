#pragma once

#include <cstdint>

namespace render {

// Stubbed WebGPU context so we don't depend on emscripten/webgpu.h yet.
class WebGPUContext {
public:
  static WebGPUContext& Get();

  // Called once during Engine::init()
  void init();

  // Called once after init – lets us remember a nominal size
  void configure(int width, int height);

  // Optional resize hook
  void resize(int width, int height);

  // Frame boundaries – currently no-op
  void beginFrame();
  void endFrame();

  bool isReady() const { return initialized; }

private:
  WebGPUContext()  = default;
  ~WebGPUContext() = default;

  WebGPUContext(const WebGPUContext&) = delete;
  WebGPUContext& operator=(const WebGPUContext&) = delete;

  bool initialized = false;
  int  width       = 0;
  int  height      = 0;
};

} // namespace render
