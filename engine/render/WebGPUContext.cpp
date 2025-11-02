#include "render/WebGPUContext.h"
#include <cstdio>
#include <cstring>   // for std::strlen if needed

using namespace render;

// Small helper: WGPUStringView from C-string
static inline WGPUStringView sv(const char* s) {
  return WGPUStringView{ s, (size_t)std::strlen(s) };
}

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext g;
  return g;
}

void WebGPUContext::Init()
{
  if (initialized) return;

  instance = wgpuCreateInstance(nullptr);

  // Create a surface bound to the HTML canvas with id="canvas"
  // emdawnwebgpu uses the CanvasId chain struct + StringView
  WGPUSurfaceDescriptor sd{};
  WGPUSurfaceDescriptorFromCanvasHTMLCanvasId canvasDesc{};
  canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLCanvasId;
  canvasDesc.canvas = sv("canvas");

  // Note: in emdawnwebgpu headers, nextInChain may be NON-const
  sd.nextInChain = (WGPUChainedStruct*)&canvasDesc;

  surface = wgpuInstanceCreateSurface(instance, &sd);

  // Device/queue
  // (For now use the default device getter; if you moved to requestAdapter/device,
  //  keep that path instead.)
  device = emscripten_webgpu_get_device();
  queue  = wgpuDeviceGetQueue(device);

  // Choose a surface format. Preferred-format helper differs across headers,
  // so just pick the common BGRA8Unorm which is the usual swapchain format on web.
  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  // Configure once with a default size; Engine will call ConfigureSurface with real size.
  WGPUSurfaceConfiguration cfg{};
  cfg.device        = device;
  cfg.format        = surfaceFormat;
  cfg.usage         = WGPUTextureUsage_RenderAttachment;
  cfg.alphaMode     = WGPUCompositeAlphaMode_Auto;
  cfg.viewFormats   = nullptr;
  cfg.viewFormatCount = 0;
  cfg.width  = 800;
  cfg.height = 600;
  wgpuSurfaceConfigure(surface, &cfg);

  // Minimal pipeline state will be created lazily when first draw happens.
  initialized = true;
}

void WebGPUContext::ConfigureSurface(int width, int height)
{
  if (!surface || !device) return;

  WGPUSurfaceConfiguration cfg{};
  cfg.device        = device;
  cfg.format        = surfaceFormat;
  cfg.usage         = WGPUTextureUsage_RenderAttachment;
  cfg.alphaMode     = WGPUCompositeAlphaMode_Auto;
  cfg.viewFormats   = nullptr;
  cfg.viewFormatCount = 0;
  cfg.width  = (uint32_t)width;
  cfg.height = (uint32_t)height;

  wgpuSurfaceConfigure(surface, &cfg);
}

WGPUTextureView WebGPUContext::BeginFrame()
{
  // Get the current drawable
  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);

  if (!st.texture || st.status != WGPUSurfaceGetCurrentTextureStatus_Success) {
    // Try once more (some browsers transiently report non-ready)
    wgpuSurfaceGetCurrentTexture(surface, &st);
    if (!st.texture || st.status != WGPUSurfaceGetCurrentTextureStatus_Success) {
      return nullptr;
    }
  }

  WGPUTextureViewDescriptor tvd{};
  tvd.aspect = WGPUTextureAspect_All;
  return wgpuTextureCreateView(st.texture, &tvd);
}

void WebGPUContext::EndFrame(WGPUTextureView view)
{
  if (view) {
    WGPUTexture tex = wgpuTextureViewGetTexture(view);
    wgpuTextureViewRelease(view);

    // Present & release
    wgpuSurfacePresent(surface);

    if (tex) {
      wgpuTextureRelease(tex);
    }
  }
}

void WebGPUContext::Shutdown()
{
  if (!initialized) return;

  if (pipeline)        { wgpuRenderPipelineRelease(pipeline); pipeline = nullptr; }
  if (shader)          { wgpuShaderModuleRelease(shader);     shader   = nullptr; }
  if (pipelineLayout)  { wgpuPipelineLayoutRelease(pipelineLayout); pipelineLayout = nullptr; }
  if (bindGroup)       { wgpuBindGroupRelease(bindGroup);     bindGroup = nullptr; }
  if (bindGroupLayout) { wgpuBindGroupLayoutRelease(bindGroupLayout); bindGroupLayout = nullptr; }
  if (mvpBuffer)       { wgpuBufferRelease(mvpBuffer);         mvpBuffer = nullptr; }

  if (surface) { wgpuSurfaceRelease(surface); surface = nullptr; }
  if (queue)   { wgpuQueueRelease(queue);     queue   = nullptr; }
  if (device)  { wgpuDeviceRelease(device);   device  = nullptr; }
  if (instance){ wgpuInstanceRelease(instance); instance = nullptr; }

  initialized = false;
}
