#if defined(FE_WEBGPU)

#include "WebGPUContext.h"
#include <string.h>
#include <stdint.h>

#if __has_include(<emscripten/emscripten.h>)
  #include <emscripten/emscripten.h>
  #define FE_HAS_EM 1
#else
  #define FE_HAS_EM 0
#endif

#if FE_HAS_EM && __has_include(<emscripten/html5_webgpu.h>)
  #include <emscripten/html5_webgpu.h>
  #define FE_HAS_SURFACE 1
#else
  #define FE_HAS_SURFACE 0
#endif

extern "C" WGPUDevice emscripten_webgpu_get_device(void);

#if FE_HAS_EM
EM_JS(int, fe_webgpu_supported, (), {
  return (typeof navigator !== 'undefined' && navigator.gpu) ? 1 : 0;
});
EM_JS(void, fe_webgpu_log_unsupported, (), {
  console.error("[FickerEngine] WebGPU not available. Use Chrome/Edge 113+ or Safari 17+. On Firefox: about:config → dom.webgpu.enabled=true.");
});
EM_ASYNC_JS(int, fe_webgpu_init_async, (), {
  if (!navigator.gpu) return 0;
  try {
    const adapter = await navigator.gpu.requestAdapter();
    if (!adapter) return 0;
    const device  = await adapter.requestDevice();

    if (typeof Module !== 'undefined') {
      Module['wgpuDevice'] = device;
      Module['preinitializedWebGPUDevice'] = device;
    }
    globalThis.wgpuDevice = device;
    globalThis.preinitializedWebGPUDevice = device;

    console.log("[FickerEngine] WebGPU device ready:", !!device);
    return 1;
  } catch (e) {
    console.error("[FickerEngine] WebGPU init failed:", e);
    return 0;
  }
});
#else
static inline int  fe_webgpu_supported() { return 0; }
static inline void fe_webgpu_log_unsupported() {}
static inline int  fe_webgpu_init_async() { return 0; }
#endif

// Simple WGSL: vertex->MVP, fragment = light gray
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
  return vec4<f32>(0.8, 0.82, 0.86, 1.0);
}
)";

WebGPUContext& WebGPUContext::Get() {
    static WebGPUContext instance;
    return instance;
}

void WebGPUContext::Init(const char* canvasSelector) {
    if (initialized) return;

    instance = wgpuCreateInstance(nullptr);
    createDevice_();
    if (!device) { fe_webgpu_log_unsupported(); return; }

    createSurface_(canvasSelector);
    if (!surface) { fe_webgpu_log_unsupported(); return; }

    surfaceFormat = WGPUTextureFormat_BGRA8Unorm;
    createMVPResources_();
    createPipeline_();

    initialized = true;
}

void WebGPUContext::ConfigureSurface(uint32_t w, uint32_t h) {
    width = w; height = h;
    if (!device || !surface) return;

    WGPUSurfaceConfiguration sc = {};
    sc.device      = device;
    sc.format      = surfaceFormat;
    sc.usage       = WGPUTextureUsage_RenderAttachment;
    sc.width       = width;
    sc.height      = height;
    sc.presentMode = WGPUPresentMode_Fifo;
    wgpuSurfaceConfigure(surface, &sc);
}

WGPUTextureView WebGPUContext::BeginFrame() {
    if (!surface) return nullptr;
    WGPUSurfaceTexture st = {};
    wgpuSurfaceGetCurrentTexture(surface, &st);
    if (!st.texture) return nullptr;
    return wgpuTextureCreateView(st.texture, nullptr);
}

void WebGPUContext::EndFrame(WGPUTextureView view) {
    if (surface) wgpuSurfacePresent(surface);
    if (view) wgpuTextureViewRelease(view);
}

// ---- internals --------------------------------------------------------------

void WebGPUContext::createDevice_() {
#if FE_HAS_EM
    if (!fe_webgpu_supported()) { device = nullptr; return; }
    if (!fe_webgpu_init_async()) { device = nullptr; return; }
#endif
    device = emscripten_webgpu_get_device();
    if (!device) return;
    queue  = wgpuDeviceGetQueue(device);
}

void WebGPUContext::createSurface_(const char* canvasSelector) {
#if FE_HAS_SURFACE
    WGPUSurfaceDescriptorFromCanvasHTMLSelector canvasDesc = {};
    canvasDesc.chain.sType = WGPUSType_SurfaceDescriptorFromCanvasHTMLSelector;
    canvasDesc.selector = canvasSelector ? canvasSelector : "#canvas";

    WGPUSurfaceDescriptor sd = {};
    sd.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&canvasDesc);
    surface = wgpuInstanceCreateSurface(instance, &sd);
#else
    (void)canvasSelector;
    surface = nullptr;
#endif
}

void WebGPUContext::createMVPResources_() {
    WGPUBufferDescriptor bd = {};
    bd.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    bd.size  = sizeof(float) * 16;
    mvpBuffer = wgpuDeviceCreateBuffer(device, &bd);

    WGPUBindGroupLayoutEntry e = {};
    e.binding     = 0;
    e.visibility  = WGPUShaderStage_Vertex;
    e.buffer.type = WGPUBufferBindingType_Uniform;

    WGPUBindGroupLayoutDescriptor bld = {};
    bld.entryCount = 1;
    bld.entries    = &e;
    bindGroupLayout = wgpuDeviceCreateBindGroupLayout(device, &bld);

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
    WGPUShaderModuleWGSLDescriptor wgsl = {};
    wgsl.chain.sType = WGPUSType_ShaderModuleWGSLDescriptor;
    wgsl.code = kWGSL;

    WGPUShaderModuleDescriptor smd = {};
    smd.nextInChain = reinterpret_cast<WGPUChainedStruct*>(&wgsl);
    shader = wgpuDeviceCreateShaderModule(device, &smd);

    WGPUVertexAttribute attr = {};
    attr.shaderLocation = 0;
    attr.offset = 0;
    attr.format = WGPUVertexFormat_Float32x3;

    WGPUVertexBufferLayout vbl = {};
    vbl.arrayStride    = sizeof(float) * 3;
    vbl.attributeCount = 1;
    vbl.attributes     = &attr;

    WGPUPipelineLayoutDescriptor pld = {};
    pld.bindGroupLayoutCount = 1;
    pld.bindGroupLayouts     = &bindGroupLayout;
    pipelineLayout = wgpuDeviceCreatePipelineLayout(device, &pld);

    WGPUColorTargetState target = {};
    target.format    = surfaceFormat;
    target.writeMask = WGPUColorWriteMask_All;

    WGPUFragmentState fs = {};
    fs.module      = shader;
    fs.entryPoint  = "fs_main";
    fs.targetCount = 1;
    fs.targets     = &target;

    WGPURenderPipelineDescriptor rp = {};
    rp.layout = pipelineLayout;
    rp.vertex.module = shader;
    rp.vertex.entryPoint = "vs_main";
    rp.vertex.bufferCount = 1;
    rp.vertex.buffers = &vbl;
    rp.fragment = &fs;
    rp.primitive.topology = WGPUPrimitiveTopology_LineList;
    rp.primitive.frontFace = WGPUFrontFace_CCW;
    rp.primitive.cullMode  = WGPUCullMode_None;
    rp.multisample.count = 1;

    pipeline = wgpuDeviceCreateRenderPipeline(device, &rp);
}

#endif // FE_WEBGPU
