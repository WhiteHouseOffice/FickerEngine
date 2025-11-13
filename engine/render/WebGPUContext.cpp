#include <cstdio>
#include "render/WebGPUContext.h"

namespace render {

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext s_instance;
  return s_instance;
}

void WebGPUContext::init() {
  if (initialized_) {
    return;
  }

#if defined(FE_WEBGPU)
  std::printf("[WebGPUContext] (stub) init() called. No real GPU yet.\n");
#else
  std::printf("[WebGPUContext] (stub) init() called with FE_WEBGPU disabled.\n");
#endif

  initialized_ = true;
}

void WebGPUContext::configure(int width, int height) {
  // In a real WebGPU backend this is where we'd create / configure the surface +
  // swapchain. For now, just store the size and log once.
#if defined(FE_WEBGPU)
  width_  = width;
  height_ = height;
  std::printf("[WebGPUContext] (stub) configure(%d, %d)\n", width, height);
#else
  (void)width;
  (void)height;
#endif
}

void WebGPUContext::resize(int width, int height) {
#if defined(FE_WEBGPU)
  width_  = width;
  height_ = height;
  std::printf("[WebGPUContext] (stub) resize(%d, %d)\n", width, height);
#else
  (void)width;
  (void)height;
#endif
}

WGPUTextureView WebGPUContext::acquireSwapchainView() {
  // Real implementation would acquire the current drawable / swapchain texture.
  // Stub: just return nullptr so callers can early-out.
#if defined(FE_WEBGPU)
  std::printf("[WebGPUContext] (stub) acquireSwapchainView() -> null\n");
#endif
  return nullptr;
}

void WebGPUContext::present() {
  // Real implementation would present the frame.
#if defined(FE_WEBGPU)
  std::printf("[WebGPUContext] (stub) present()\n");
#endif
}

} // namespace render
