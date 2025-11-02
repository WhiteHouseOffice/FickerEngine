#pragma once
#include <webgpu/webgpu.h>

namespace render {

class WebGPUContext {
public:
  static WebGPUContext& Get();

  // ... your existing public methods (Init, ConfigureSurface, etc.)

  // NEW: accessors used by RenderMesh
  WGPUDevice       GetDevice()       const { return device; }
  WGPUQueue        GetQueue()        const { return queue; }
  WGPUTextureFormat GetSurfaceFormat() const { return surfaceFormat; }
  
  static WebGPUContext& Get() {
    static WebGPUContext s;
    return s;
  }

  // Create instance/adapter/device/queue/surface
  void Init();

  // Configure surface (call after Init, and on resize)
  void Configure(int width, int height);

  // Per-frame acquire/present
  WGPUTextureView BeginFrame();
  void EndFrame(WGPUTextureView view);

  // Accessors (use these, not private fields)
  WGPUDevice        Device()        const { return device; }
  WGPUQueue         Queue()         const { return queue; }
  WGPUSurface       Surface()       const { return surface; }
  WGPUTextureFormat SurfaceFormat() const { return surfaceFormat; }

private:
  WebGPUContext() = default;

  WGPUInstance      instance      = nullptr;
  WGPUAdapter       adapter       = nullptr;
  WGPUDevice        device        = nullptr;
  WGPUQueue         queue         = nullptr;
  WGPUSurface       surface       = nullptr;
  WGPUTextureFormat surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  bool initialized = false;
};

} // namespace render
