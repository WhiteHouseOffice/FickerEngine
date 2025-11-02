#pragma once
#include <cstdint>
#include <emscripten/webgpu.h>

// Minimal WebGPU context for Emscripten.
class WebGPUContext {
public:
  static WebGPUContext& Get() { static WebGPUContext s; return s; }

  void Init(const char* canvasSelector = "#canvas");
  WGPUTextureView BeginFrame();   // acquires current swapchain view (may return nullptr)
  void EndFrame(WGPUTextureView); // presents + releases

  // Expose a few handles for the engine loop:
  WGPUDevice  device  = nullptr;
  WGPUQueue   queue   = nullptr;
  WGPUSurface surface = nullptr;
  WGPUTextureFormat format = WGPUTextureFormat_BGRA8Unorm;

private:
  WebGPUContext() = default;

  void CreateSurfaceFromCanvas(const char* selector);
  void ConfigureSurface(int width, int height);
  void EnsureConfigured();

  // helpers
  int  CanvasPixelWidth() const;
  int  CanvasPixelHeight() const;

  // state
  const char* selector_ = "#canvas";
  WGPUInstance instance = nullptr;

  int  width_       = 0;
  int  height_      = 0;
  bool configured_  = false;

  // For EndFrame release
  WGPUTexture currentTexture_ = nullptr;
};
