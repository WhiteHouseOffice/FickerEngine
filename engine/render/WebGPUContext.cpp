#include "render/WebGPUContext.h"

#if __has_include(<emscripten/html5.h>)
  #include <emscripten/html5.h>
#endif

extern "C" WGPUDevice emscripten_webgpu_get_device(void);

// JS helpers to read canvas CSS size and DPR
extern "C" {
EM_JS(double, fe_canvas_width_css, (const char* sel), {
  const s = UTF8ToString(sel);
  const c = document.querySelector(s);
  if (!c) return 0;
  const r = c.getBoundingClientRect();
  return r.width;
});
EM_JS(double, fe_canvas_height_css, (const char* sel), {
  const s = UTF8ToString(sel);
  const c = document.querySelector(s);
  if (!c) return 0;
  const r = c.getBoundingClientRect();
  return r.height;
});
EM_JS(double, fe_dpr, (), { return (self.devicePixelRatio || 1.0); });
}

void WebGPUContext::Init(const char* canvasSelector) {
  if (m_initialized) return;

  // 1) Instance (null ok for Emscripten)
  instance = wgpuCreateInstance(nullptr);

  // 2) Device/Queue
  device = emscripten_webgpu_get_device();
  if (!device) return;
  queue  = wgpuDeviceGetQueue(device);

  // 3) Surface from canvas
  CreateSurfaceFromCanvas(canvasSelector);
  if (!surface) return;

  // 4) Preferred format
  format = wgpuSurfaceGetPreferredFormat(surface, /*adapter*/ nullptr);
  if (format == WGPUTextureFormat_Undefined) {
    format = WGPUTextureFormat_BGRA8Unorm;
  }

  // 5) Initial configure
  const double cssW = fe_canvas_width_css(canvasSelector);
  const double cssH = fe_canvas_height_css(canvasSelector);
  const double dpr  = fe_dpr();
  const int pxW = (int)(cssW * dpr + 0.5);
  const int pxH = (int)(cssH * dpr + 0.5);
  ConfigureSurface(pxW > 1 ? pxW : 1, pxH > 1 ? pxH : 1);

  m_initialized = true;
}

void WebGPUContext::CreateSurfaceFromCanvas(const char* selector) {
  WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc{};
  canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
  canvasDesc.selector    = selector;

  WGPUSurfaceDescriptor surfDesc{};
  surfDesc.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&canvasDesc);

  surface = wgpuInstanceCreateSurface(instance, &surfDesc);
}

void WebGPUContext::ConfigureSurface(int width, int height) {
  if (!surface || !device) return;

  WGPUSurfaceConfiguration cfg{};
  cfg.device       = device;
  cfg.format       = format;
  cfg.width        = (uint32_t)width;
  cfg.height       = (uint32_t)height;
  cfg.usage        = WGPUTextureUsage_RenderAttachment;
  cfg.alphaMode    = WGPUCompositeAlphaMode_Auto;
  // cfg.presentMode = WGPUPresentMode_Fifo; // (optional) default/Auto is fine

  wgpuSurfaceConfigure(surface, &cfg);

  m_width      = width;
  m_height     = height;
  m_configured = true;
}

void WebGPUContext::EnsureConfigured() {
  // Recompute desired size from CSS * DPR
  const double cssW = fe_canvas_width_css("#canvas");
  const double cssH = fe_canvas_height_css("#canvas");
  const double dpr  = fe_dpr();
  const int pxW = (int)(cssW * dpr + 0.5);
  const int pxH = (int)(cssH * dpr + 0.5);

  if (!m_configured || pxW != m_width || pxH != m_height) {
    ConfigureSurface(pxW > 1 ? pxW : 1, pxH > 1 ? pxH : 1);
  }
}

WGPUTextureView WebGPUContext::BeginFrame() {
  if (!device || !queue || !surface) return nullptr;
  EnsureConfigured();

  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);
  if (!st.texture || st.status != WGPUSurfaceGetCurrentTextureStatus_Success) {
    // If lost, try reconfigure once and retry
    m_configured = false;
    EnsureConfigured();
    wgpuSurfaceGetCurrentTexture(surface, &st);
    if (!st.texture || st.status != WGPUSurfaceGetCurrentTextureStatus_Success)
      return nullptr;
  }

  WGPUTextureView view = wgpuTextureCreateView(st.texture, nullptr);
  // Note: we must present the texture in EndFrame()
  return view;
}

void WebGPUContext::EndFrame(WGPUTextureView view) {
  if (view) {
    // Present the surface texture via present() on the surface
    // On Emscripten, presentation is implicit on releasing the view + calling present.
    wgpuTextureViewRelease(view);
  }
  // Finish presentation
  wgpuSurfacePresent(surface);
}
