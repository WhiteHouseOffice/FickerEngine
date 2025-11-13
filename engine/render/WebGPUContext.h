#pragma once
#include <cstdint>

namespace render {

// Minimal fake WebGPU handle types so we don't depend on system WebGPU headers.
// This keeps the rest of the engine code compiling on any Emscripten version.
#if defined(FE_WEBGPU)
using WGPUInstance          = void*;
using WGPUAdapter           = void*;
using WGPUDevice            = void*;
using WGPUQueue             = void*;
using WGPUSurface           = void*;
using WGPUTextureView       = void*;
using WGPUTextureFormat     = std::uint32_t;
#else
using WGPUTextureView       = void*;
#endif

class WebGPUContext {
public:
  // Singleton accessor
  static WebGPUContext& Get();

  // Lifecycle
  void init();                     // called once from Engine
  void configure(int width, int height); // placeholder for swapchain config
  void resize(int width, int height);    // placeholder for resize handling

  // Frame handling (all stubs for now)
  WGPUTextureView acquireSwapchainView();
  void present();

  bool isReady() const { return initialized_; }

private:
  WebGPUContext()  = default;
  ~WebGPUContext() = default;

  bool initialized_ = false;

#if defined(FE_WEBGPU)
  // "Handles" are just void* right now, but we keep the layout
  // so we can plug in real WebGPU later without touching users.
  WGPUInstance      instance_       = nullptr;
  WGPUAdapter       adapter_        = nullptr;
  WGPUDevice        device_         = nullptr;
  WGPUQueue         queue_          = nullptr;
  WGPUSurface       surface_        = nullptr;
  WGPUTextureFormat swapchainFormat_ = 0;
  int               width_          = 0;
  int               height_         = 0;
#endif
};

} // namespace render
