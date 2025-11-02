#if defined(FE_WEBGPU)

#include "WebGPUContext.h"
#include <string.h>
#include <stdint.h>

#if __has_include(<emscripten/emscripten.h>)
  #include <emscripten/emscripten.h>
  #define FE_HAS_EM 1
#else
  #define FE_HAS_EM 0
#endif

// Provided by the Emscripten runtime when using -sUSE_WEBGPU=1
extern "C" WGPUDevice emscripten_webgpu_get_device(void);

#if FE_HAS_EM
// --- WebGPU availability check ---
EM_JS(int, fe_webgpu_supported, (), {
  return (typeof navigator !== 'undefined' && navigator.gpu) ? 1 : 0;
});

// --- Log missing WebGPU support ---
EM_JS(void, fe_webgpu_log_unsupported, (), {
  console.error("[FickerEngine] WebGPU not available. Use Chrome/Edge 113+ or Safari 17+. "
                + "On Firefox: about:config → dom.webgpu.enabled=true.");
});

// --- Async adapter + device creation (requires -sASYNCIFY) ---
EM_ASYNC_JS(int, fe_webgpu_init_async, (), {
  if (!navigator.gpu) return 0;
  try {
    const adapter = await navigator.gpu.requestAdapter();
    if (!adapter) return 0;
    const device  = await adapter.requestDevice();
    Module.wgpuDevice = device; // Emscripten expects this global
    return 1;
  } catch (e) {
    console.error("[FickerEngine] WebGPU init failed:", e);
    return 0;
  }
});
#else
// --- Non-Emscripten stubs (native builds) ---
static inline int  fe_webgpu_supported() { return 0; }
static inline void fe_webgpu_log_unsupported() {}
static inline int  fe_webgpu_init_async() { return 0; }
#endif

// ============================================================================
// WebGPUContext Implementation
// ============================================================================

WebGPUContext& WebGPUContext::Get() {
    static WebGPUContext instance;
    return instance;
}

void WebGPUContext::Init(const char* canvasSelector) {
    if (initialized_) return;
    createDevice_();
    if (!device) return;
    createSurface_(canvasSelector);
    initialized_ = true;
}

void WebGPUContext::createDevice_() {
#if FE_HAS_EM
    if (!fe_webgpu_supported()) {
        device = nullptr;
        fe_webgpu_log_unsupported();
        return;
    }
    if (!fe_webgpu_init_async()) {
        device = nullptr;
        fe_webgpu_log_unsupported();
        return;
    }
#endif
    device = emscripten_webgpu_get_device();
    if (!device) {
        fe_webgpu_log_unsupported();
        return;
    }
    queue = wgpuDeviceGetQueue(device);
}

void WebGPUContext::createSurface_(const char* canvasSelector) {
#if FE_HAS_EM && __has_include(<emscripten/html5_webgpu.h>)
    #include <emscripten/html5_webgpu.h>
    WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc = {};
    canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
    canvasDesc.selector = canvasSelector ? canvasSelector : "#canvas";
    WGPUSurfaceDescriptor surfaceDesc = {};
    surfaceDesc.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&canvasDesc);
    surface = wgpuInstanceCreateSurface(instance, &surfaceDesc);
#else
    (void)canvasSelector;
#endif
}

void WebGPUContext::ConfigureSurface(int width, int height) {
    if (!device || !surface) return;

    WGPUSurfaceConfiguration config = {};
    config.device = device;
    config.format = surfaceFormat;
    config.usage = WGPUTextureUsage_RenderAttachment;
    config.width = static_cast<uint32_t>(width);
    config.height = static_cast<uint32_t>(height);
    wgpuSurfaceConfigure(surface, &config);
}

WGPUTextureView WebGPUContext::BeginFrame() {
    if (!surface) return nullptr;
    WGPUSurfaceTexture surfaceTex;
    wgpuSurfaceGetCurrentTexture(surface, &surfaceTex);
    return surfaceTex.texture
        ? wgpuTextureCreateView(surfaceTex.texture, nullptr)
        : nullptr;
}

void WebGPUContext::EndFrame(WGPUTextureView view) {
    if (surface) wgpuSurfacePresent(surface);
    if (view) wgpuTextureViewRelease(view);
}

#endif // FE_WEBGPU
