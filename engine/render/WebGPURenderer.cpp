
#include "IRenderer.h"
#ifdef FE_WEB
#include <webgpu/webgpu_cpp.h>
#include <emscripten/html5_webgpu.h>
#include <cmath>
#include <cstdio>
#include <memory>

class WebGPURenderer : public IRenderer {
  wgpu::Device device;
  wgpu::Queue queue;
  wgpu::SwapChain swapchain;
  int width=0, height=0;
public:
  bool init(int w, int h) override {
    width=w; height=h;
    WGPUDevice dev = emscripten_webgpu_get_device();
    if(!dev) { printf("WebGPU device not available\\n"); return false; }
    device = wgpu::Device::Acquire(dev);
    queue = device.GetQueue();

    // Configure swapchain on the default canvas
    wgpu::SwapChainDescriptor scDesc{};
    scDesc.usage = wgpu::TextureUsage::RenderAttachment;
    scDesc.format = wgpu::TextureFormat::BGRA8Unorm;
    scDesc.width = (uint32_t)width;
    scDesc.height = (uint32_t)height;
    scDesc.presentMode = wgpu::PresentMode::Fifo;
    swapchain = device.CreateSwapChain(nullptr, &scDesc);
    return true;
  }

  void beginFrame() override {}

  void drawTestScene(double t) override {
    float r = 0.5f + 0.5f*std::sin(t*0.7);
    float g = 0.5f + 0.5f*std::sin(t*1.1 + 2.0);
    float b = 0.5f + 0.5f*std::sin(t*1.7 + 4.0);

    wgpu::TextureView backbuf = swapchain.GetCurrentTextureView();
    wgpu::CommandEncoder enc = device.CreateCommandEncoder();
    wgpu::RenderPassColorAttachment colorAttach{};
    colorAttach.view = backbuf;
    colorAttach.loadOp = wgpu::LoadOp::Clear;
    colorAttach.storeOp = wgpu::StoreOp::Store;
    colorAttach.clearValue = { r, g, b, 1.0f };
    wgpu::RenderPassDescriptor rpDesc{};
    rpDesc.colorAttachmentCount = 1;
    rpDesc.colorAttachments = &colorAttach;
    wgpu::RenderPassEncoder pass = enc.BeginRenderPass(&rpDesc);
    pass.End();
    wgpu::CommandBuffer cmd = enc.Finish();
    queue.Submit(1, &cmd);
    (void)backbuf;
  }

  void endFrame() override {}
};

std::unique_ptr<IRenderer> CreateWebRenderer() {
  return std::make_unique<WebGPURenderer>();
}
#endif
