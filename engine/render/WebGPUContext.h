#pragma once
#include <cstdint>
#include <webgpu/webgpu.h>

namespace render {

class WebGPUContext {
public:
  // Singleton
  static WebGPUContext& Get();

  // Lifetime
  void Init();                       // create instance/adapter/device/queue/surface
  void Configure(int width, int height); // (re)configure the swapchain/surface

  // Frame
  WGPUTextureView BeginFrame();
  void            EndFrame(WGPUTextureView view);

  // Accessors (names kept for compatibility with existing call sites)
  WGPUDevice        Device()        const { return device; }
  WGPUQueue         Queue()         const { return queue; }
  WGPUTextureFormat SurfaceFormat() const { return surfaceFormat; }
  int               Width()         const { return width; }
  int               Height()        const { return height; }

  // Also provide Get* aliases in case other files use them
  WGPUDevice        GetDevice()        const { return device; }
  WGPUQueue         GetQueue()         const { return queue; }
  WGPUTextureFormat GetSurfaceFormat() const { return surfaceFormat; }
  int               GetWidth()         const { return width; }
  int               GetHeight()        const { return height; }

private:
  WebGPUContext() = default;
  ~WebGPUContext() = default;

  // State
  bool              initialized   = false;

  // GPU objects
  WGPUInstance      instance      = nullptr;
  WGPUAdapter       adapter       = nullptr;
  WGPUDevice        device        = nullptr;
  WGPUQueue         queue         = nullptr;
  WGPUSurface       surface       = nullptr;
  WGPUTextureFormat surfaceFormat = WGPUTextureFormat_Undefined;

  // Backbuffer size
  int               width         = 0;
  int               height        = 0;
};

} // namespace render
