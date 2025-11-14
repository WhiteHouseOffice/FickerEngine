#include "render/WebGPUContext.h"
#include <cstdio>

namespace render {

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext s_ctx;
  return s_ctx;
}

void WebGPUContext::init() {
  if (initialized) return;
  std::puts("[WebGPUContext] init (stub)");
  initialized = true;
}

void WebGPUContext::configure(int w, int h) {
  width  = w;
  height = h;
  std::printf("[WebGPUContext] configure %dx%d (stub)\n", w, h);
}

WGPUTextureView WebGPUContext::acquireSwapchainView() {
  // In the real backend this would return a real texture view.
  // For now, nullptr is fine.
  return nullptr;
}

void WebGPUContext::present(WGPUTextureView) {
  // Stub: nothing to do.
}

void WebGPUContext::resize(int w, int h) {
  width  = w;
  height = h;
  std::printf("[WebGPUContext] resize %dx%d (stub)\n", w, h);
}

} // namespace render
