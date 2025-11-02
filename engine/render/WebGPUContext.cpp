#include "render/WebGPUContext.h"

#include <cstring>
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

extern "C" WGPUDevice emscripten_webgpu_get_device(void);

// --- JS helpers to read CSS size and DPR ---
extern "C" {
EM_JS(double, fe_css_w, (const char* sel), {
  const s = UTF8ToString(sel);
  const c = document.querySelector(s);
  if (!c) return 0;
  const r = c.getBoundingClientRect();
  return r.width;
});
EM_JS(double, fe_css_h, (const char* sel), {
  const s = UTF8ToString(sel);
  const c = document.querySelector(s);
  if (!c) return 0;
  const r = c.getBoundingClientRect();
  return r.height;
});
EM_JS(double, fe_dpr, (), { return (self.devicePixelRatio || 1.0); });
}

void WebGPUContext::Init(const char* canvasSelector) {
  if (device) return;

  selector_ = canvasSelector ? canvasSelector : "#canvas";

  // Instance (optional/ok null in Emscripten)
  instance = wgpuCreateInstance(nullptr);

  // Device + queue
  device = emscripten_webgpu_get_device();
  if (!device) return;
  queue = wgpuDeviceGetQueue(device);

  // Surface
  CreateSurfaceFromCanvas(selector_);
  if (!surface) return;

  // Format
  format = wgpuSurfaceGetPreferredFormat(surface, /*adapter*/ nullptr);
  if (format == WGPUTextureFormat_Undefined) {
    format = WGPUTextureFormat_BGRA8Unorm;
  }

  // Set the canvas pixel size (attributes!) to CSS * DPR.
  const int pxW = CanvasPixelWidth();
  const int pxH = CanvasPixelHeight();
  emscripten_set_canvas_element_size(selector_, pxW > 1 ? pxW : 1, pxH > 1 ? pxH : 1);

  // Configure surface
  ConfigureSurface(pxW > 1 ? pxW : 1, pxH > 1 ? pxH : 1);
}

void WebGPUContext::CreateSurfaceFromCanvas(const char* sel) {
  WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc{};
  canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
  canvasDesc.selector    = sel;

  WGPUSurfaceDescriptor sd{};
  sd.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&canvasDesc);

  surface = wgpuInstanceCreateSurface(instance, &sd);
}

int WebGPUContext::CanvasPixelWidth() const {
  const double css = fe_css_w(selector_);
  const double dpr = fe_dpr();
  const int px = (int)(css * dpr + 0.5);
  return px > 0 ? px : 1;
}
int WebGPUContext::CanvasPixelHeight() const {
  const double css = fe_css_h(selector_);
  const double dpr = fe_dpr();
  const int px = (int)(css * dpr + 0.5);
  return px > 0 ? px : 1;
}

void WebGPUContext::ConfigureSurface(int w, int h) {
  if (!surface || !device) return;

  // Only set fields guaranteed by emscripten/webgpu.h
  WGPUSurfaceConfiguration cfg;
  std::memset(&cfg, 0, sizeof(cfg));
  cfg.device = device;
  cfg.format = format;
  cfg.usage  = WGPUTextureUsage_RenderAttachment;
  cfg.width  = (uint32_t)w;
  cfg.height = (uint32_t)h;

  wgpuSurfaceConfigure(surface, &cfg);

  width_      = w;
  height_     = h;
  configured_ = true;
}

void WebGPUContext::EnsureConfigured() {
  const int pxW = CanvasPixelWidth();
  const int pxH = CanvasPixelHeight();

  // Keep canvas element size in sync with CSS*DPR (important on zoom/resize).
  emscripten_set_canvas_element_size(selector_, pxW, pxH);

  if (!configured_ || pxW != width_ || pxH != height_) {
    ConfigureSurface(pxW, pxH);
  }
}

WGPUTextureView WebGPUContext::BeginFrame() {
  if (!device || !queue || !surface) return nullptr;

  EnsureConfigured();

  // Acquire
  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);

  // If not configured or lost, try once more after reconfiguring
  if (!st.texture || st.status != WGPUSurfaceGetCurrentTextureStatus_Success) {
    configured_ = false;
    EnsureConfigured();
    wgpuSurfaceGetCurrentTexture(surface, &st);
    if (!st.texture || st.status != WGPUSurfaceGetCurrentTextureStatus_Success) {
      return nullptr;
    }
  }

  currentTexture_ = st.texture;
  WGPUTextureView view = wgpuTextureCreateView(st.texture, nullptr);
  return view;
}

void WebGPUContext::EndFrame(WGPUTextureView view) {
  if (view) {
    wgpuTextureViewRelease(view);
  }
  if (currentTexture_) {
    // Present and release the acquired texture
    wgpuSurfacePresent(surface);
    wgpuTextureRelease(currentTexture_);
    currentTexture_ = nullptr;
  }
}
