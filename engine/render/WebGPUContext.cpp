#include "WebGPUContext.h"
#include <cstdio>
#include <cstring>

namespace render {

// Dawn-port callback signatures (note WGPUStringView + 2 userdatas)
static void OnRequestAdapter(WGPURequestAdapterStatus status,
                             WGPUAdapter received,
                             WGPUStringView message,
                             void* userdata1,
                             void* /*userdata2*/)
{
  if (status == WGPURequestAdapterStatus_Success) {
    *reinterpret_cast<WGPUAdapter*>(userdata1) = received;
  } else {
    std::printf("[FickerEngine] RequestAdapter failed: %.*s\n",
                (int)message.length, message.data ? message.data : "");
  }
}

static void OnRequestDevice(WGPURequestDeviceStatus status,
                            WGPUDevice received,
                            WGPUStringView message,
                            void* userdata1,
                            void* /*userdata2*/)
{
  if (status == WGPURequestDeviceStatus_Success) {
    *reinterpret_cast<WGPUDevice*>(userdata1) = received;
  } else {
    std::printf("[FickerEngine] RequestDevice failed: %.*s\n",
                (int)message.length, message.data ? message.data : "");
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

  // Adapter (Dawn port: callback info has mode/callback/userdata1/2)
  {
    WGPURequestAdapterCallbackInfo acb{};
    acb.mode      = WGPUCallbackMode_AllowSpontaneous; // compile-safe with port
    acb.callback  = OnRequestAdapter;
    acb.userdata1 = &adapter;
    acb.userdata2 = nullptr;

    (void)wgpuInstanceRequestAdapter(instance, /*options*/ nullptr, acb);
  }

  if (!adapter) {
    std::printf("[FickerEngine] No adapter (callback did not return one yet).\n");
    // We continue; on some builds the callback fires immediately before this line.
  }

  // Device
  {
    WGPUDeviceDescriptor dd{};
    WGPURequestDeviceCallbackInfo dcb{};
    dcb.mode      = WGPUCallbackMode_AllowSpontaneous;
    dcb.callback  = OnRequestDevice;
    dcb.userdata1 = &device;
    dcb.userdata2 = nullptr;

    (void)wgpuAdapterRequestDevice(adapter, &dd, dcb);
  }

  if (!device) {
    std::printf("[FickerEngine] No device yet (waiting for callback).\n");
    // Not fatal here; queue retrieval will be null until callback hits.
  } else {
    queue = wgpuDeviceGetQueue(device);
  }

  // Surface: with emdawnwebgpu we can create with an empty descriptor
  // and it binds to the default canvas Emscripten provides.
  {
    WGPUSurfaceDescriptor sd{};
    sd.nextInChain = nullptr;
    surface = wgpuInstanceCreateSurface(instance, &sd);
    if (!surface) {
      std::printf("[FickerEngine] Failed to create surface.\n");
      return;
    }
  }

  initialized = true;
  std::printf("[FickerEngine] WebGPU (Dawn port) initialized.\n");
}

void WebGPUContext::Configure(int width, int height)
{
  if (!initialized || !surface || !device) return;

  // If device just arrived via callback now, make sure we have queue.
  if (!queue) queue = wgpuDeviceGetQueue(device);

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

  // If device/queue are not ready yet (callbacks not fired), skip a frame.
  if (!device || !queue) return nullptr;

  WGPUSurfaceTexture st{};
  wgpuSurfaceGetCurrentTexture(surface, &st);

  if (!st.texture) {
    // Could be hidden tab / unconfigured surface; skip.
    return nullptr;
  }

  WGPUTextureViewDescriptor vd{};
  return wgpuTextureCreateView(st.texture, &vd);
}

void WebGPUContext::EndFrame(WGPUTextureView view)
{
  if (!initialized || !surface) return;
  wgpuSurfacePresent(surface);
  if (view) wgpuTextureViewRelease(view);
}

} // namespace render
