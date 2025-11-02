#pragma once
#include <cstdint>

// Dawn (emdawnwebgpu) header first
#if __has_include(<webgpu/webgpu.h>)
  #include <webgpu/webgpu.h>
#elif __has_include(<emscripten/webgpu.h>)
  // very old SDKs
  #include <emscripten/webgpu.h>
#else
  #error "WebGPU headers not found (tried <webgpu/webgpu.h> and <emscripten/webgpu.h>)"
#endif

class WebGPUContext {
public:
  static WebGPUContext& Get() { static WebGPUContext s; return s; }

  void Init(const char* canvasSelector = "#canvas");
  void EnsureConfigured();

  // Begin/End present frame. If surface is not available, returns nullptr and render should no-op.
  WGPUTextureView BeginFrame();
  void EndFrame(WGPUTextureView view);

  // Public bits used by renderer
  WGPUInstance       instance = nullptr;
  WGPUDevice         device   = nullptr;
  WGPUQueue          queue    = nullptr;
  WGPUSurface        surface  = nullptr;
  WGPUTextureFormat  format   = WGPUTextureFormat_BGRA8Unorm; // default for canvas

  // Current swap texture (if any)
  WGPUTexture        currentTexture = nullptr;

  // Canvas metrics (in device pixels)
  int width  = 0;
  int height = 0;

private:
  WebGPUContext() = default;

  void CreateSurfaceFromCanvas(const char* selector);
  void ConfigureSurface(int w, int h);
  int  CanvasPixelWidth() const;
  int  CanvasPixelHeight() const;

  const char* selector = "#canvas";
  bool configured = false;
};
