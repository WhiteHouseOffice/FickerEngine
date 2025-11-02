#include "render/WebGPUContext.h"

// NOTE: This file only ensures symbols used by Engine.cpp / RenderMesh.cpp exist.
// The internals of Init/Configure/BeginFrame/EndFrame can be filled/updated to the
// exact Dawn/emdawnwebgpu API you’re using next.

namespace render {

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext s;
  return s;
}

void WebGPUContext::Init() {
  if (initialized) return;

  // Minimal skeleton; fill with your actual adapter/device/surface creation.
  // (Keeping no-ops here avoids compile/link errors until we wire Dawn calls.)
  instance = nullptr;
  adapter  = nullptr;
  device   = nullptr;
  queue    = nullptr;
  surface  = nullptr;
  surfaceFormat = WGPUTextureFormat_BGRA8Unorm; // common default; adjust later
  initialized = true;
}

void WebGPUContext::Configure(int w, int h) {
  width  = w;
  height = h;
  // In your real implementation, call wgpuDeviceCreateSurface / configure surface here.
}

WGPUTextureView WebGPUContext::BeginFrame() {
  // Return a null view for now; your real code will grab current texture view.
  return nullptr;
}

void WebGPUContext::EndFrame(WGPUTextureView /*view*/) {
  // Submit/present in your real implementation.
}

} // namespace render
