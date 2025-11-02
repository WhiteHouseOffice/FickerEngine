#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "math/MiniMath.h"
#include "render/WebGPUContext.h"
#include <webgpu/webgpu.h>

// ... inside render_frame():
extern struct GPUData {
  WGPUBuffer vbo; WGPUBuffer ibo; uint32_t indexCount;
  WGPUIndexFormat indexFormat; WGPUPrimitiveTopology topology;
} g_data;

WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(ctx.device, nullptr);

WGPURenderPassColorAttachment color = {};
color.view = backView;
color.loadOp  = WGPULoadOp_Clear;
color.storeOp = WGPUStoreOp_Store;
color.clearValue = {0.05f,0.05f,0.06f,1.0f};

WGPURenderPassDescriptor rp = {};
rp.colorAttachmentCount = 1;
rp.colorAttachments     = &color;

WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(enc, &rp);
wgpuRenderPassEncoderSetPipeline(pass, ctx.pipeline);
wgpuRenderPassEncoderSetBindGroup(pass, 0, ctx.bindGroup, 0, nullptr);
wgpuRenderPassEncoderSetVertexBuffer(pass, 0, g_data.vbo, 0, WGPU_WHOLE_SIZE);
wgpuRenderPassEncoderSetIndexBuffer(pass, g_data.ibo, g_data.indexFormat, 0, WGPU_WHOLE_SIZE);
wgpuRenderPassEncoderDrawIndexed(pass, g_data.indexCount, 1, 0, 0, 0);
wgpuRenderPassEncoderEnd(pass);

WGPUCommandBuffer cmds = wgpuCommandEncoderFinish(enc, nullptr);
wgpuQueueSubmit(ctx.queue, 1, &cmds);
