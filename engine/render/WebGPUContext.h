#pragma once

namespace render {

class WebGPUContext {
public:
  static WebGPUContext& Get();
  void init();
};

} // namespace render
