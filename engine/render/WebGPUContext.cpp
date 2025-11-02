#if defined(FE_WEBGPU)

#include "WebGPUContext.h"
#include <string.h>
#include <stdint.h>

// Resilient WebGPU headers are handled in WebGPUContext.h

// --- Emscripten integration (for EM_JS) ---
#if __has_include(<emscripten/emscripten.h>)
  #include <emscripten/emscripten.h>
  #define FE_HAS_EM 1
#else
  #define FE_HAS_EM 0
#endif

// Emscripten runtime device getter (declared in JS runtime)
extern "C" WGPUDevice emscripten_webgpu_get_device(void);

// JS-side feature detection (guarded so native builds still compile)
#if FE_HAS_EM
EM_ASYNC_JS(int, fe_webgpu_init_async, (), {
  if (!navigator.gpu) return 0;
  try {
    const adapter = await navigator.gpu.requestAdapter();
    if (!adapter) return 0;
    const device  = await adapter.requestDevice();
    Module.wgpuDevice = device; // Emscripten uses this internally
    return 1;
  } catch (e) {
    console.error("[FickerEngine] WebGPU init failed:", e);
    return 0;
  }
});
#endif

// -------- Minimal WGSL (positions only + MVP; fixed color) -------------------
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

// -------- Singleton ----------------------------------------------------------
WebGPUContext& WebGPUContext::Get() {
    static WebGPUContext ctx;
    return ctx;
}

// -------- Public API ---------------------------------------------------------
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

void WebGPUContext::ConfigureSurface(uint32_t w, uint32_t h) {
    width = w; height = h;
    configureSurface_();
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

// -------- Internals ----------------------------------------------------------
void WebGPUContext::createDevice_() {
#if FE_HAS_EM
    if (!fe_webgpu_supported()) { device = nullptr; fe_webgpu_log_unsupported(); return; }
    // Ensure JS side created the device (await)
    if (!fe_webgpu_init_async()) { device = nullptr; fe_webgpu_log_unsupported(); return; }
#endif
    // Safe to call now
    device = emscripten_webgpu_get_device();
    if (!device) { fe_webgpu_log_unsupported(); return; }
    queue  = wgpuDeviceGetQueue(device);
}

void WebGPUContext::createSurface_(const char* canvasSelector) {
#if FE_HAS_EM_WEBGPU
    // Create a surface from a Canvas element by CSS selector (default "#canvas")
    WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc = {};
    canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
    canvasDesc.selector    = canvasSelector;

    WGPUSurfaceDescriptor surfDesc = {};
    surfDesc.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&canvasDesc);

    surface = wgpuInstanceCreateSurface(instance, &surfDesc);
#else
    // If the runner didn't expose emscripten/webgpu.h, surface creation
    // via selector won't be available. Fail clearly (assertions are on).
    surface = nullptr;
#endif
}

void WebGPUContext::configureSurface_() {
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
    // Uniform buffer for a 4x4 matrix (16 floats)
    WGPUBufferDescriptor bd = {};
    bd.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    bd.size  = sizeof(float) * 16;
    mvpBuffer = wgpuDeviceCreateBuffer(device, &bd);

    // Bind group layout: uMVP @group(0) @binding(0)
    WGPUBindGroupLayoutEntry entry = {};
    entry.binding    = 0;
    entry.visibility = WGPUShaderStage_Vertex;
    entry.buffer.type = WGPUBufferBindingType_Uniform;

    WGPUBindGroupLayoutDescriptor bld = {};
    bld.entryCount = 1;
    bld.entries    = &entry;
    bindGroupLayout = wgpuDeviceCreateBindGroupLayout(device, &bld);

    // Bind group
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
    // WGSL module
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

    // Pipeline layout (bind group 0 -> MVP)
    WGPUPipelineLayoutDescriptor pld = {};
    pld.bindGroupLayoutCount = 1;
    pld.bindGroupLayouts     = &bindGroupLayout;
    pipelineLayout = wgpuDeviceCreatePipelineLayout(device, &pld);

    // Fragment state (one color attachment)
    WGPUColorTargetState color = {};
    color.format    = surfaceFormat;
    color.writeMask = WGPUColorWriteMask_All;

    WGPUFragmentState fs = {};
    fs.module      = shader;
    fs.entryPoint  = "fs_main";
    fs.targetCount = 1;
    fs.targets     = &color;

    // Render pipeline (LineList for grid; triangles also supported)
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

#endif // FE_WEBGPU
