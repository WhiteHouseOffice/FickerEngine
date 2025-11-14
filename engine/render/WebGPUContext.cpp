#include "render/WebGPUContext.h"
#include <cstdio>

namespace render {

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext inst;
  return inst;
}

void WebGPUContext::init() {
  if (initialized) {
    return;
  }
  initialized = true;
  std::printf("[WebGPUContext] stub init (no real GPU yet)\n");
}

void WebGPUContext::configure(int w, int h) {
  width  = w;
  height = h;
  std::printf("[WebGPUContext] stub configure %dx%d\n", w, h);
}

void WebGPUContext::resize(int w, int h) {
  width  = w;
  height = h;
}

void WebGPUContext::beginFrame() {
  // In a real backend, we'd acquire a swapchain image here
}

void WebGPUContext::endFrame() {
  // In a real backend, we'd present here
}

} // namespace render
