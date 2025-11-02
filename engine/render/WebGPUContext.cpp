#include "render/WebGPUContext.h"
#include <cstring>    // std::memset
#include <cstdio>     // std::printf

namespace render {

// ---- helper callbacks for adapter/device requests --------------------------------

static void OnAdapterRequested(WGPURequestAdapterStatus status,
                               WGPUAdapter received,
                               const char* message,
                               void* pUserData)
{
  if (status == WGPURequestAdapterStatus_Success) {
    *reinterpret_cast<WGPUAdapter*>(pUserData) = received;
  } else {
    std::printf("[FickerEngine] Adapter request failed: %s\n", message ? message : "(no message)");
  }
}

static void OnDeviceRequested(WGPURequestDeviceStatus status,
                              WGPUDevice received,
                              const char* message,
                              void* pUserData)
{
  if (status == WGPURequestDeviceStatus_Success) {
    *reinterpret_cast<WGPUDevice*>(pUserData) = received;
  } else {
    std::printf("[FickerEngine] Device request failed: %s\n", message ? message : "(no message)");
  }
}

// ----------------------------------------------------------------------------------

void WebGPUContext::Init()
{
  if (initialized) return;

  // Instance (no special options needed)
  instance = wgpuCreateInstance(nullptr);
  if (!instance) {
    std::printf("[FickerEngine] wgpuCreateInstance failed.\n");
    return;
  }

  // Request adapter
  wgpuInstanceRequestAdapter(instance, /*options*/ nullptr,
                             OnAdapterRequested, &adapter);
  if (!adapter) {
    std::printf("[FickerEngine] No adapter available.\n");
    return;
  }

  // Request device
  wgpuAdapterRequestDevice(adapter, /*desc*/ nullptr,
                           OnDeviceRequested, &device);
  if (!device) {
    std::printf("[FickerEngine] No device available.\n");
    return;
  }

  queue = wgpuDeviceGetQueue(device);

  // Create a surface. On Emscripten/Dawn, creating the surface with an empty
  // descriptor is supported and binds to the default <canvas id="canvas">.
  // (We avoid HTML selector/canvas-id chained structs that are not present in
  // emdawnwebgpu's C headers.)
  WGPUSurfaceDescriptor sd{};
  sd.nextInChain = nullptr;
  surface = wgpuInstanceCreateSurface(instance, &sd);

  if (!surface) {
    std::printf("[FickerEngine] Failed to create surface.\n");
    return;
  }

  initialized = true;
  std::printf("[FickerEngine] WebGPU init OK (Dawn port).\n");
}

void WebGPUContext::Configure(int width, int height)
{
  if (!initialized || !device || !surface) return;

  // Minimal surface configuration (no alpha mode/present mode fiddling)
  WGPUSurfaceConfiguration cfg{};
  cfg.device      = device;
  cfg.format      = surfaceFormat; // BGRA8Unorm default works in browsers
  cfg.usage       = WGPUTextureUsage_RenderAttachment;
  cfg.width       = static_cast<uint32_t>(width > 0 ? width : 1);
  cfg.height      = static_cast<uint32_t>(height > 0 ? height : 1);
  cfg.presentMode = WGPUPresentMode_Fifo;

  wgpuSurfaceConfigure(surface, &cfg);
}

WGPUTextureView WebGPUContext::BeginFrame()
{
  if (!initialized || !surface) return nullptr;

  // Acquire the current surface texture (do NOT rely on status enum names)
  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);

  if (!st.texture) {
    // If acquisition failed (tab hidden / resize race), skip this frame
    return nullptr;
  }

  WGPUTextureViewDescriptor vd{};
  WGPUTextureView view = wgpuTextureCreateView(st.texture, &vd);

  // We keep st.texture alive until present; Dawn presents/releases internally on Present()
  return view;
}

void WebGPUContext::EndFrame(WGPUTextureView view)
{
  if (!initialized || !surface) return;

  // Present and drop the view
  wgpuSurfacePresent(surface);
  if (view) {
    wgpuTextureViewRelease(view);
  }
}

} // namespace render
