#pragma once

// We are on Emscripten with the Dawn WebGPU port.
// Use the Dawn header location:
#include <webgpu/webgpu.h>

namespace render {

class WebGPUContext {
public:
  static WebGPUContext& Get();

  void Init();
  void ConfigureSurface(int width, int height);
  void Shutdown();

  // Per-frame acquire/present
  WGPUTextureView BeginFrame();
  void EndFrame(WGPUTextureView view);

  // Public state used by renderer
  WGPUInstance      instance = nullptr;
  WGPUDevice        device   = nullptr;
  WGPUQueue         queue    = nullptr;
  WGPUSurface       surface  = nullptr;
  WGPUTextureFormat surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  // Minimal pipeline bits (optional; created lazily elsewhere)
  WGPUBuffer            mvpBuffer       = nullptr;
  WGPUBindGroupLayout   bindGroupLayout = nullptr;
  WGPUBindGroup         bindGroup       = nullptr;
  WGPUPipelineLayout    pipelineLayout  = nullptr;
  WGPUShaderModule      shader          = nullptr;
  WGPURenderPipeline    pipeline        = nullptr;

  bool initialized = false;

private:
  WebGPUContext() = default;
  WebGPUContext(const WebGPUContext&) = delete;
  WebGPUContext& operator=(const WebGPUContext&) = delete;
};

} // namespace render
