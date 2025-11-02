#pragma once

// Dawn WebGPU C header provided by the emdawnwebgpu port
#include <webgpu/webgpu.h>

namespace render {

// Minimal singleton context used by Engine
class WebGPUContext {
public:
  static WebGPUContext& Get() {
    static WebGPUContext s;
    return s;
  }

  // Initialize instance, adapter, device, surface, queue
  void Init();

  // (Re)configure the surface (call once after Init, or when canvas size changes)
  void Configure(int width, int height);

  // Acquire a frame's view. Returns nullptr if no texture is available.
  WGPUTextureView BeginFrame();

  // Present and clean up (releases the view)
  void EndFrame(WGPUTextureView view);

  // Quick getters
  WGPUDevice Device() const { return device; }
  WGPUQueue  Queue()  const { return queue; }
  WGPUTextureFormat SurfaceFormat() const { return surfaceFormat; }

private:
  WebGPUContext() = default;

  // WebGPU handles
  WGPUInstance      instance       = nullptr;
  WGPUAdapter       adapter        = nullptr;
  WGPUDevice        device         = nullptr;
  WGPUQueue         queue          = nullptr;
  WGPUSurface       surface        = nullptr;
  WGPUTextureFormat surfaceFormat  = WGPUTextureFormat_BGRA8Unorm; // safe default

  bool initialized = false;
};

} // namespace render
