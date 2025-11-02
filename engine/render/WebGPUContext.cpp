#include "render/WebGPUContext.h"

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

#if __has_include(<emscripten/html5_webgpu.h>)
  #include <emscripten/html5_webgpu.h> // optional helper for surface-from-canvas
#endif

#include <cstdio>
#include <cstring>

extern "C" WGPUDevice emscripten_webgpu_get_device(void);

// JS helpers for DPR and logging
EM_JS(double, fe_dpr, (), {
  return (typeof window !== 'undefined' && window.devicePixelRatio) ? window.devicePixelRatio : 1.0;
});
EM_JS(void, fe_log, (const char* cstr), {
  // best-effort small logger
  if (typeof UTF8ToString === 'function') console.log('[FE]', UTF8ToString(cstr));
});

// ———————————————————————————————————————————————————————————————————

void WebGPUContext::Init(const char* canvasSelector) {
  selector = canvasSelector;

  // Create instance (nullptr ok in Emscripten / Dawn shim)
  instance = wgpuCreateInstance(nullptr);

  // Get device synchronously from Emscripten
  device = emscripten_webgpu_get_device();
  if (!device) {
    fe_log("WebGPU device is null (emscripten_webgpu_get_device failed).");
    return;
  }
  queue = wgpuDeviceGetQueue(device);

  // Try to create a surface from canvas (if helper header exists)
#if __has_include(<emscripten/html5_webgpu.h>)
  {
    WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc{};
    canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
    canvasDesc.selector = selector;

    WGPUSurfaceDescriptor sd{};
    sd.nextInChain = reinterpret_cast<const WGPUChainedStruct*>(&canvasDesc);
    surface = wgpuInstanceCreateSurface(instance, &sd);
  }
#else
  surface = nullptr; // we will still compile; BeginFrame will no-op if no surface
#endif

  // Choose a default format (BGRA8 is the canvas format in browsers)
  format = WGPUTextureFormat_BGRA8Unorm;

  EnsureConfigured();
}

int WebGPUContext::CanvasPixelWidth() const {
  double cssW = 0.0, cssH = 0.0;
  // If the element is missing, fall back to 1280x720
  if (EMSCRIPTEN_RESULT_SUCCESS != emscripten_get_element_css_size(selector, &cssW, &cssH)) {
    cssW = 1280.0;
    cssH = 720.0;
  }
  double dpr = fe_dpr();
  return (int)(cssW * dpr);
}

int WebGPUContext::CanvasPixelHeight() const {
  double cssW = 0.0, cssH = 0.0;
  if (EMSCRIPTEN_RESULT_SUCCESS != emscripten_get_element_css_size(selector, &cssW, &cssH)) {
    cssW = 1280.0;
    cssH = 720.0;
  }
  double dpr = fe_dpr();
  return (int)(cssH * dpr);
}

void WebGPUContext::ConfigureSurface(int w, int h) {
  if (!surface || !device) return;

  // Ensure canvas backing size tracks DPR
  emscripten_set_canvas_element_size(selector, w, h);

  WGPUSurfaceConfiguration cfg{};
  cfg.device = device;
  cfg.format = format;
  cfg.usage  = WGPUTextureUsage_RenderAttachment;
  cfg.width  = w;
  cfg.height = h;
  wgpuSurfaceConfigure(surface, &cfg);
  configured = true;
}

void WebGPUContext::EnsureConfigured() {
  width  = CanvasPixelWidth();
  height = CanvasPixelHeight();
  if (surface && width > 0 && height > 0) {
    ConfigureSurface(width, height);
  }
}

WGPUTextureView WebGPUContext::BeginFrame() {
  if (!surface || !device) {
    return nullptr; // headless/no-op
  }

  // If canvas size changed (resizing), reconfigure
  int w = CanvasPixelWidth();
  int h = CanvasPixelHeight();
  if (w != width || h != height || !configured) {
    ConfigureSurface(w, h);
  }

  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);
  if (!st.texture) {
    // Surface may not be ready yet; skip this frame
    return nullptr;
  }

  currentTexture = st.texture;
  WGPUTextureViewDescriptor tvd{};
  return wgpuTextureCreateView(currentTexture, &tvd);
}

void WebGPUContext::EndFrame(WGPUTextureView /*view*/) {
  if (!surface || !currentTexture) return;

  // Present the surface texture and release our reference
  wgpuSurfacePresent(surface);
  wgpuTextureRelease(currentTexture);
  currentTexture = nullptr;
}
