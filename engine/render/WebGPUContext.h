#pragma once
#include <cstdint>
#include <webgpu/webgpu.h>

namespace render {

class WebGPUContext {
public:
  // Singleton access (defined in .cpp)
  static WebGPUContext& Get();

  // Create device/queue/surface and choose surfaceFormat
  void Init();

  // (Re)configure the surface when size changes
  void ConfigureSurface(int width, int height);

  // Frame lifecycle around a render pass
  WGPUTextureView BeginFrame();
  void            EndFrame(WGPUTextureView view);

  // Read-only accessors used by other systems (e.g., RenderMesh)
  WGPUDevice        GetDevice()        const { return device; }
  WGPUQueue         GetQueue()         const { return queue; }
  WGPUTextureFormat GetSurfaceFormat() const { return surfaceFormat; }
  int               GetWidth()         const { return width; }
  int               GetHeight()        const { return height; }

private:
  // Only the singleton can construct/destroy
  WebGPUContext() = default;
  ~WebGPUContext() = default;

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
