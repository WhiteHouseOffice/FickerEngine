#pragma once

namespace render {

class WebGPUContext {
public:
  static WebGPUContext& Get();

  // Create graphics context (WebGL on web, stub on native).
  void init();

private:
  WebGPUContext() = default;
  bool initialized_ = false;
};

} // namespace render
