#pragma once
#include <cstdint>

#if defined(__EMSCRIPTEN__)
  #include <emscripten/webgpu.h>
#else
  // Not building with Emscripten -> don't try to include its headers.
  // You can either include a native WebGPU header (Dawn) here later,
  // or just stop compiling WebGPU files on native builds.
  #error "WebGPUContext requires Emscripten. Build with emcmake/em++ or exclude this file on native builds."
#endif

class WebGPUContext {
public:
  static WebGPUContext& Get() { static WebGPUContext s; return s; }
  void Init(const char* canvasSelector = "#canvas");
  WGPUTextureView BeginFrame();
  void EndFrame(WGPUTextureView);

  WGPUDevice  device  = nullptr;
  WGPUQueue   queue   = nullptr;
  WGPUSurface surface = nullptr;
  WGPUTextureFormat format = WGPUTextureFormat_BGRA8Unorm;

private:
  WebGPUContext() = default;
  void CreateSurfaceFromCanvas(const char* selector);
  void ConfigureSurface(int width, int height);
  void EnsureConfigured();

  int  CanvasPixelWidth() const;
  int  CanvasPixelHeight() const;

  const char* selector_ = "#canvas";
  WGPUInstance instance = nullptr;

  int  width_ = 0, height_ = 0;
  bool configured_ = false;

  WGPUTexture currentTexture_ = nullptr;
};
