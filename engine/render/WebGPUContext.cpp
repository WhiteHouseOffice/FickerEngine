#include "render/WebGPUContext.h"
#include <cstdio>
#include <cstring>

namespace render {

// Request callbacks (Dawn port style)
static void OnRequestAdapter(WGPURequestAdapterStatus status,
                             WGPUAdapter received,
                             char const* message,
                             void* userdata)
{
  if (status == WGPURequestAdapterStatus_Success) {
    *reinterpret_cast<WGPUAdapter*>(userdata) = received;
  } else {
    std::printf("[FickerEngine] RequestAdapter failed: %s\n", message ? message : "(no message)");
  }
}

static void OnRequestDevice(WGPURequestDeviceStatus status,
                            WGPUDevice received,
                            char const* message,
                            void* userdata)
{
  if (status == WGPURequestDeviceStatus_Success) {
    *reinterpret_cast<WGPUDevice*>(userdata) = received;
  } else {
    std::printf("[FickerEngine] RequestDevice failed: %s\n", message ? message : "(no message)");
  }
}

void WebGPUContext::Init()
{
  if (initialized) return;

  // Instance
  instance = wgpuCreateInstance(nullptr);
  if (!instance) {
    std::printf("[FickerEngine] wgpuCreateInstance failed.\n");
    return;
  }

  // Adapter (blocking callback mode)
  WGPURequestAdapterCallbackInfo acb{};
  acb.mode     = WGPUCallbackMode_Blocking;
  acb.callback = OnRequestAdapter;
  acb.userdata = &adapter;

  (void)wgpuInstanceRequestAdapter(instance, /*options*/ nullptr, acb);
  if (!adapter) {
    std::printf("[FickerEngine] No adapter.\n");
    return;
  }

  // Device (blocking callback mode)
  WGPUDeviceDescriptor dd{};
  WGPURequestDeviceCallbackInfo dcb{};
  dcb.mode     = WGPUCallbackMode_Blocking;
  dcb.callback = OnRequestDevice;
  dcb.userdata = &device;

  (void)wgpuAdapterRequestDevice(adapter, &dd, dcb);
  if (!device) {
    std::printf("[FickerEngine] No device.\n");
    return;
  }

  queue = wgpuDeviceGetQueue(device);

  // Surface: with emdawnwebgpu we can pass an empty descriptor and it binds
  // to the default canvas that Emscripten provides.
  WGPUSurfaceDescriptor sd{};
  sd.nextInChain = nullptr;
  surface = wgpuInstanceCreateSurface(instance, &sd);
  if (!surface) {
    std::printf("[FickerEngine] Failed to create surface.\n");
    return;
  }

  initialized = true;
  std::printf("[FickerEngine] WebGPU (Dawn port) initialized.\n");
}

void WebGPUContext::Configure(int width, int height)
{
  if (!initialized || !device || !surface) return;

  WGPUSurfaceConfiguration cfg{};
  cfg.device      = device;
  cfg.format      = surfaceFormat;
  cfg.usage       = WGPUTextureUsage_RenderAttachment;
  cfg.width       = width  > 0 ? (uint32_t)width  : 1;
  cfg.height      = height > 0 ? (uint32_t)height : 1;
  cfg.presentMode = WGPUPresentMode_Fifo;

  wgpuSurfaceConfigure(surface, &cfg);
}

WGPUTextureView WebGPUContext::BeginFrame()
{
  if (!initialized || !surface) return nullptr;

  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);

  if (!st.texture) {
    // Could be hidden tab / resize race; just skip drawing this frame.
    return nullptr;
  }

  WGPUTextureViewDescriptor vd{};
  WGPUTextureView view = wgpuTextureCreateView(st.texture, &vd);
  return view;
}

void WebGPUContext::EndFrame(WGPUTextureView view)
{
  if (!initialized || !surface) return;
  wgpuSurfacePresent(surface);
  if (view) wgpuTextureViewRelease(view);
}

} // namespace render
