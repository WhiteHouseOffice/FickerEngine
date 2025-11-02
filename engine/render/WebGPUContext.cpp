#if defined(FE_WEBGPU)

#include "WebGPUContext.h"
#include <string.h>

// Minimal WGSL: positions only + MVP; fixed color
static const char* kWGSL = R"(
struct MVP { mvp : mat4x4<f32> };
@group(0) @binding(0) var<uniform> uMVP : MVP;

struct VSIn { @location(0) aPos : vec3<f32>; };
struct VSOut { @builtin(position) pos : vec4<f32>; };

@vertex
fn vs_main(input : VSIn) -> VSOut {
  var out : VSOut;
  out.pos = uMVP.mvp * vec4<f32>(input.aPos, 1.0);
  return out;
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
  return vec4<f32>(0.85, 0.85, 0.9, 1.0);
}
)";

WebGPUContext& WebGPUContext::Get() {
    static WebGPUContext ctx;
    return ctx;
}

void WebGPUContext::Init(const char* canvasSelector) {
    if (initialized) return;

    instance = wgpuCreateInstance(nullptr);
    createDevice_();
    createSurface_(canvasSelector);
    configureSurface_();
    createMVPResources_();
    createPipeline_();

    initialized = true;
}

void WebGPUContext::createDevice_() {
    // Emscripten provides a default device
    WGPUDevice dev = emscripten_webgpu_get_device();
    device = dev;
    queue  = wgpuDeviceGetQueue(device);
}

void WebGPUContext::createSurface_(const char* canvasSelector) {
    WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc = {};
    canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
    canvasDesc.selector = canvasSelector;

    WGPUSurfaceDescriptor surfDesc = {};
    surfDesc.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&canvasDesc);

    surface = wgpuInstanceCreateSurface(instance, &surfDesc);
}

void WebGPUContext::ConfigureSurface(uint32_t w, uint32_t h) {
    width = w; height = h;
    configureSurface_();
}

void WebGPUContext::configureSurface_() {
    // BGRA8Unorm is widely supported in browsers
    surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

    WGPUSurfaceConfiguration sc = {};
    sc.device      = device;
    sc.format      = surfaceFormat;
    sc.usage       = WGPUTextureUsage_RenderAttachment;
    sc.width       = width;
    sc.height      = height;
    sc.presentMode = WGPUPresentMode_Fifo;

    wgpuSurfaceConfigure(surface, &sc);
}

void WebGPUContext::createMVPResources_() {
    // 16 floats for mat4
    WGPUBufferDescriptor bd = {};
    bd.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    bd.size  = sizeof(float) * 16;
    mvpBuffer = wgpuDeviceCreateBuffer(device, &bd);

    // BindGroupLayout
    WGPUBindGroupLayoutEntry entry = {};
    entry.binding    = 0;
    entry.visibility = WGPUShaderStage_Vertex;
    entry.buffer.type = WGPUBufferBindingType_Uniform;

    WGPUBindGroupLayoutDescriptor bld = {};
    bld.entryCount = 1;
    bld.entries    = &entry;
    bindGroupLayout = wgpuDeviceCreateBindGroupLayout(device, &bld);

    // BindGroup
    WGPUBindGroupEntry be = {};
    be.binding = 0;
    be.buffer  = mvpBuffer;
    be.offset  = 0;
    be.size    = sizeof(float) * 16;

    WGPUBindGroupDescriptor bgd = {};
    bgd.layout     = bindGroupLayout;
    bgd.entryCount = 1;
    bgd.entries    = &be;
    bindGroup = wgpuDeviceCreateBindGroup(device, &bgd);
}

void WebGPUContext::createPipeline_() {
    // WGSL shader
    WGPUShaderModuleWGSLDescriptor wgsl = {};
    wgsl.chain.sType = WGPUSType_ShaderModuleWGSLDescriptor;
    wgsl.code = kWGSL;

    WGPUShaderModuleDescriptor smd = {};
    smd.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&wgsl);
    shader = wgpuDeviceCreateShaderModule(device, &smd);

    // Vertex layout: vec3<f32> at location 0
    WGPUVertexAttribute attr = {};
    attr.shaderLocation = 0;
    attr.offset = 0;
    attr.format = WGPUVertexFormat_Float32x3;

    WGPUVertexBufferLayout vbl = {};
    vbl.arrayStride    = sizeof(float) * 3;
    vbl.attributeCount = 1;
    vbl.attributes     = &attr;

    // Pipeline layout
    WGPUPipelineLayoutDescriptor pld = {};
    pld.bindGroupLayoutCount = 1;
    pld.bindGroupLayouts     = &bindGroupLayout;
    pipelineLayout = wgpuDeviceCreatePipelineLayout(device, &pld);

    // Fragment state
    WGPUColorTargetState color = {};
    color.format    = surfaceFormat;
    color.writeMask = WGPUColorWriteMask_All;

    WGPUFragmentState fs = {};
    fs.module     = shader;
    fs.entryPoint = "fs_main";
    fs.targetCount= 1;
    fs.targets    = &color;

    // Render pipeline
    WGPURenderPipelineDescriptor rp = {};
    rp.layout = pipelineLayout;
    rp.vertex.module = shader;
    rp.vertex.entryPoint = "vs_main";
    rp.vertex.bufferCount = 1;
    rp.vertex.buffers = &vbl;

    rp.fragment = &fs;

    rp.primitive.topology = WGPUPrimitiveTopology_LineList;
    rp.primitive.stripIndexFormat = WGPUIndexFormat_Undefined;
    rp.primitive.frontFace = WGPUFrontFace_CCW;
    rp.primitive.cullMode  = WGPUCullMode_None;

    rp.multisample.count = 1;

    pipeline = wgpuDeviceCreateRenderPipeline(device, &rp);
}

WGPUTextureView WebGPUContext::BeginFrame() {
    WGPUSurfaceTexture st = {};
    wgpuSurfaceGetCurrentTexture(surface, &st);
    if (!st.texture) return nullptr;

    WGPUTextureViewDescriptor vd = {};
    return wgpuTextureCreateView(st.texture, &vd);
}

void WebGPUContext::EndFrame(WGPUTextureView /*view*/) {
    wgpuSurfacePresent(surface);
}

#endif // FE_WEBGPU
