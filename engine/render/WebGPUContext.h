#pragma once

#if __has_include(<emscripten/webgpu.h>)
  #include <emscripten/webgpu.h>
#elif __has_include(<webgpu/webgpu.h>)
  #include <webgpu/webgpu.h>
#else
  #error "No WebGPU headers found"
#endif

#include <cstdint>

class WebGPUContext {
public:
  static WebGPUContext& Get() { static WebGPUContext s; return s; }

  // Initialize with a CSS selector (default emscripten template uses "#canvas")
  void Init(const char* canvasSelector = "#canvas");

  // Acquire the swapchain texture view for this frame (configures if needed)
  WGPUTextureView BeginFrame();

  // Present and release per-frame resources
  void EndFrame(WGPUTextureView view);

  // Public state (read-only from outside)
  WGPUInstance       instance = nullptr;
  WGPUDevice         device   = nullptr;
  WGPUQueue          queue    = nullptr;
  WGPUSurface        surface  = nullptr;
  WGPUTextureFormat  format   = WGPUTextureFormat_BGRA8Unorm;

private:
  WebGPUContext() = default;

  void CreateSurfaceFromCanvas(const char* selector);
  void ConfigureSurface(int width, int height);
  void EnsureConfigured();

  // Size & state
  int  m_width  = 0;
  int  m_height = 0;
  bool m_configured = false;
  bool m_initialized = false;
};
