#include "render/WebGPUContext.h"

namespace render {

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext s;
  return s;
}

void WebGPUContext::init() {
  // Native-only build: WebGPU/WebGL/Emscripten is removed.
  // Keep as a stub so Engine::init() can call it safely.
}

} // namespace render
