#if defined(FE_WEBGPU)

#include "WebGPUContext.h"
#include <emscripten/html5_webgpu.h> // emscripten_webgpu_get_device
#include <webgpu/webgpu_cpp.h>
#include <cstring>

// Tiny WGSL shader (positions only + MVP; solid per-draw color via push constants later if needed)
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
  // Light grey by default; caller can swap pipeline later if we add colors
  return vec4<f32>(0.85, 0.85, 0.9, 1.0);
}
)";

WebGPUContext& WebGPUContext::Get() {
    static WebGPUContext ctx;
    return ctx;
}

void WebGPUContext::Init(const char* canvasSelector) {
    if (initialized) return;
    instance = wgpu::CreateInstance({});
    createDevice_();
    createSurface_(canvasSelector);
    configureSurface_();
    createMVPResources_();
    createPipeline_();
    initialized = true;
}

void WebGPUContext::createDevice_() {
    // Emscripten gives us a pre-initialized WebGPU device
    WGPUDevice dev = emscripten_webgpu_get_device();
    device = wgpu::Device::Acquire(dev);
    queue  = device.GetQueue();
}

void WebGPUContext::createSurface_(const char* canvasSelector) {
    // Create surface from the default canvas (Emscripten default shell has #canvas)
    wgpu::SurfaceDescriptorFromCanvasHTMLSelector canvasDesc{};
    canvasDesc.selector = canvasSelector;

    wgpu::SurfaceDescriptor sd{};
    sd.nextInChain = &canvasDesc;
    surface = instance.CreateSurface(&sd);
}

void WebGPUContext::ConfigureSurface(uint32_t w, uint32_t h) {
    width = w; height = h;
    configureSurface_();
}

void WebGPUContext::configureSurface_() {
    // Pick preferred format
    surfaceFormat = (wgpu::TextureFormat)surface.GetPreferredFormat(/*adapter*/nullptr);
    wgpu::SurfaceConfiguration sc{};
    sc.usage = wgpu::TextureUsage::RenderAttachment;
    sc.format = surfaceFormat;
    sc.width  = width;
    sc.height = height;
    sc.presentMode = wgpu::PresentMode::Fifo;
    surface.Configure(&sc);
}

void WebGPUContext::createMVPResources_() {
    // 16 floats
    wgpu::BufferDescriptor bd{};
    bd.size  = sizeof(float) * 16;
    bd.usage = wgpu::BufferUsage::Uniform | wgpu::BufferUsage::CopyDst;
    mvpBuffer = device.CreateBuffer(&bd);

    // Bind group layout
    wgpu::BindGroupLayoutEntry e{};
    e.binding    = 0;
    e.visibility = wgpu::ShaderStage::Vertex;
    e.buffer.type = wgpu::BufferBindingType::Uniform;

    wgpu::BindGroupLayoutDescriptor bld{};
    bld.entryCount = 1;
    bld.entries    = &e;
    bindGroupLayout = device.CreateBindGroupLayout(&bld);

    // Bind group
    wgpu::BindGroupEntry be{};
    be.binding = 0;
    be.buffer  = mvpBuffer;
    be.offset  = 0;
    be.size    = sizeof(float) * 16;

    wgpu::BindGroupDescriptor bgd{};
    bgd.layout     = bindGroupLayout;
    bgd.entryCount = 1;
    bgd.entries    = &be;
    bindGroup = device.CreateBindGroup(&bgd);
}

void WebGPUContext::createPipeline_() {
    // Shader
    wgpu::ShaderModuleWGSLDescriptor wgsl{};
    wgsl.code = kWGSL;

    wgpu::ShaderModuleDescriptor smd{};
    smd.nextInChain = &wgsl;
    shader = device.CreateShaderModule(&smd);

    // Vertex layout: vec3<f32> at location 0
    wgpu::VertexAttribute attr{};
    attr.shaderLocation = 0;
    attr.offset = 0;
    attr.format = wgpu::VertexFormat::Float32x3;

    wgpu::VertexBufferLayout vbl{};
    vbl.arrayStride = sizeof(float)*3;
    vbl.attributeCount = 1;
    vbl.attributes = &attr;

    // Pipeline layout
    wgpu::PipelineLayoutDescriptor pld{};
    pld.bindGroupLayoutCount = 1;
    pld.bindGroupLayouts = &bindGroupLayout;
    pipelineLayout = device.CreatePipelineLayout(&pld);

    // Render pipeline
    wgpu::RenderPipelineDescriptor rp{};
    rp.layout = pipelineLayout;

    rp.vertex.module = shader;
    rp.vertex.entryPoint = "vs_main";
    rp.vertex.bufferCount = 1;
    rp.vertex.buffers = &vbl;

    wgpu::FragmentState fs{};
    fs.module = shader;
    fs.entryPoint = "fs_main";

    wgpu::ColorTargetState color{};
    color.format = surfaceFormat;
    color.writeMask = wgpu::ColorWriteMask::All;

    fs.targetCount = 1;
    fs.targets = &color;
    rp.fragment = &fs;

    rp.primitive.topology = wgpu::PrimitiveTopology::LineList; // our grid/marker are lines
    rp.primitive.stripIndexFormat = wgpu::IndexFormat::Undefined;
    rp.primitive.frontFace = wgpu::FrontFace::CCW;
    rp.primitive.cullMode = wgpu::CullMode::None;

    rp.multisample.count = 1;

    pipeline = device.CreateRenderPipeline(&rp);
}

wgpu::TextureView WebGPUContext::BeginFrame() {
    wgpu::SurfaceTexture st = surface.GetCurrentTexture();
    if (!st.texture) {
        return nullptr;
    }
    wgpu::TextureViewDescriptor vd{};
    auto view = st.texture.CreateView(&vd);
    return view;
}

void WebGPUContext::EndFrame(const wgpu::TextureView& /*view*/) {
    surface.Present();
}

#endif // FE_WEBGPU
