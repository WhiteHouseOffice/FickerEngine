#pragma once
#include <cstdint>

// ────────────────────────────────────────────────────────────
// Make the header self-sufficient for editors / native builds.
// On web (Emscripten) we include the real header.
// Else we provide dummy typedefs + structs so IntelliSense is happy.
// ────────────────────────────────────────────────────────────
#if defined(__EMSCRIPTEN__)
  #include <emscripten/webgpu.h>
#else
  // Handles
  typedef void* WGPUInstance;
  typedef void* WGPUDevice;
  typedef void* WGPUQueue;
  typedef void* WGPUSurface;
  typedef void* WGPUTexture;
  typedef void* WGPUTextureView;
  typedef void* WGPUShaderModule;
  typedef void* WGPURenderPipeline;
  typedef void* WGPUBuffer;
  typedef void* WGPUBindGroupLayout;
  typedef void* WGPUBindGroup;
  typedef void* WGPUPipelineLayout;

  // Basic scalar/enums as uints (values are placeholders for editor only)
  typedef uint32_t WGPUTextureFormat;
  typedef uint32_t WGPUPresentMode;
  typedef uint32_t WGPUTextureUsageFlags;
  typedef uint32_t WGPUBufferUsageFlags;
  typedef uint32_t WGPUShaderStageFlags;
  typedef uint32_t WGPUBufferBindingType;
  typedef uint32_t WGPUColorWriteMaskFlags;
  typedef uint32_t WGPUPrimitiveTopology;
  typedef uint32_t WGPUVertexFormat;
  typedef uint32_t WGPUFrontFace;
  typedef uint32_t WGPUCullMode;
  typedef uint32_t WGPUIndexFormat;

  // Constants used by our .cpp (dummy values for editor)
  #ifndef WGPUTextureFormat_BGRA8Unorm
  #define WGPUTextureFormat_BGRA8Unorm 87u
  #endif
  #ifndef WGPUPresentMode_Fifo
  #define WGPUPresentMode_Fifo 2u
  #endif
  #ifndef WGPUTextureUsage_RenderAttachment
  #define WGPUTextureUsage_RenderAttachment 0x10u
  #endif
  #ifndef WGPUBufferUsage_Uniform
  #define WGPUBufferUsage_Uniform 0x10u
  #endif
  #ifndef WGPUBufferUsage_CopyDst
  #define WGPUBufferUsage_CopyDst 0x20u
  #endif
  #ifndef WGPUBufferUsage_Vertex
  #define WGPUBufferUsage_Vertex 0x1u
  #endif
  #ifndef WGPUBufferUsage_Index
  #define WGPUBufferUsage_Index 0x2u
  #endif
  #ifndef WGPUShaderStage_Vertex
  #define WGPUShaderStage_Vertex 0x1u
  #endif
  #ifndef WGPUBufferBindingType_Uniform
  #define WGPUBufferBindingType_Uniform 0u
  #endif
  #ifndef WGPUColorWriteMask_All
  #define WGPUColorWriteMask_All 0xFu
  #endif
  #ifndef WGPUPrimitiveTopology_LineList
  #define WGPUPrimitiveTopology_LineList 1u
  #endif
  #ifndef WGPUVertexFormat_Float32x3
  #define WGPUVertexFormat_Float32x3 3u
  #endif
  #ifndef WGPUFrontFace_CCW
  #define WGPUFrontFace_CCW 0u
  #endif
  #ifndef WGPUCullMode_None
  #define WGPUCullMode_None 0u
  #endif
  #ifndef WGPUIndexFormat_Uint32
  #define WGPUIndexFormat_Uint32 2u
  #endif

  // Chained struct base (dummy)
  typedef struct WGPUChainedStruct {
    const struct WGPUChainedStruct* next;
    uint32_t sType;
  } WGPUChainedStruct;

  // Surface config/texture (subset)
  typedef struct WGPUSurfaceConfiguration {
    WGPUDevice device;
    WGPUTextureFormat format;
    WGPUTextureUsageFlags usage;
    uint32_t width, height;
    WGPUPresentMode presentMode;
    const WGPUChainedStruct* nextInChain;
  } WGPUSurfaceConfiguration;

  typedef struct WGPUSurfaceTexture {
    WGPUTexture texture;
    uint32_t status;
    const WGPUChainedStruct* suboptimal;
  } WGPUSurfaceTexture;

  // Buffer/bind group descriptors (subset)
  typedef struct WGPUBufferDescriptor {
    const WGPUChainedStruct* nextInChain;
    const char* label;
    uint64_t size;
    WGPUBufferUsageFlags usage;
    bool mappedAtCreation;
  } WGPUBufferDescriptor;

  typedef struct WGPUBindGroupLayoutEntry {
    const WGPUChainedStruct* nextInChain;
    uint32_t binding;
    WGPUShaderStageFlags visibility;
    struct { WGPUBufferBindingType type; bool hasDynamicOffset; uint64_t minBindingSize; } buffer;
  } WGPUBindGroupLayoutEntry;

  typedef struct WGPUBindGroupLayoutDescriptor {
    const WGPUChainedStruct* nextInChain;
    const char* label;
    uint32_t entryCount;
    const WGPUBindGroupLayoutEntry* entries;
  } WGPUBindGroupLayoutDescriptor;

  typedef struct WGPUBindGroupEntry {
    const WGPUChainedStruct* nextInChain;
    uint32_t binding;
    WGPUBuffer buffer;
    uint64_t offset;
    uint64_t size;
  } WGPUBindGroupEntry;

  typedef struct WGPUBindGroupDescriptor {
    const WGPUChainedStruct* nextInChain;
    const char* label;
    WGPUBindGroupLayout layout;
    uint32_t entryCount;
    const WGPUBindGroupEntry* entries;
  } WGPUBindGroupDescriptor;

  // Shader/pipeline descriptors (subset)
  typedef struct WGPUShaderModuleWGSLDescriptor {
    WGPUChainedStruct chain;
    const char* code;
  } WGPUShaderModuleWGSLDescriptor;

  typedef struct WGPUShaderModuleDescriptor {
    const WGPUChainedStruct* nextInChain;
    const char* label;
  } WGPUShaderModuleDescriptor;

  typedef struct WGPUVertexAttribute {
    WGPUVertexFormat format;
    uint64_t offset;
    uint32_t shaderLocation;
  } WGPUVertexAttribute;

  typedef struct WGPUVertexBufferLayout {
    uint64_t arrayStride;
    uint32_t stepMode;
    uint32_t attributeCount;
    const WGPUVertexAttribute* attributes;
  } WGPUVertexBufferLayout;

  typedef struct WGPUPipelineLayoutDescriptor {
    const WGPUChainedStruct* nextInChain;
    const char* label;
    uint32_t bindGroupLayoutCount;
    const WGPUBindGroupLayout* bindGroupLayouts;
  } WGPUPipelineLayoutDescriptor;

  typedef struct WGPUColorTargetState {
    WGPUTextureFormat format;
    uint32_t blend;               // placeholder
    WGPUColorWriteMaskFlags writeMask;
  } WGPUColorTargetState;

  typedef struct WGPUFragmentState {
    WGPUShaderModule module;
    const char* entryPoint;
    uint32_t constantCount;
    const void* constants;        // placeholder
    uint32_t targetCount;
    const WGPUColorTargetState* targets;
  } WGPUFragmentState;

  typedef struct WGPURenderPipelineDescriptor {
    const WGPUChainedStruct* nextInChain;
    const char* label;
    WGPUPipelineLayout layout;
    struct { WGPUShaderModule module; const char* entryPoint; uint32_t bufferCount; const WGPUVertexBufferLayout* buffers; } vertex;
    struct { WGPUPrimitiveTopology topology; WGPUFrontFace frontFace; WGPUCullMode cullMode; uint32_t unclippedDepth; uint32_t stripIndexFormat; } primitive;
    uint32_t depthStencil; // placeholder
    struct { uint32_t count; uint32_t mask; uint32_t alphaToCoverageEnabled; } multisample;
    const WGPUFragmentState* fragment;
  } WGPURenderPipelineDescriptor;

#endif // !__EMSCRIPTEN__

// ────────────────────────────────────────────────────────────
// Public interface
// ────────────────────────────────────────────────────────────
class WebGPUContext {
public:
  static WebGPUContext& Get();

  void Init(const char* canvasSelector = "#canvas");
  void ConfigureSurface(uint32_t w, uint32_t h);
  WGPUTextureView BeginFrame();
  void EndFrame(WGPUTextureView view);

  // Exposed handles (used by renderer/engine)
  WGPUInstance       instance = nullptr;
  WGPUDevice         device   = nullptr;
  WGPUQueue          queue    = nullptr;
  WGPUSurface        surface  = nullptr;
  WGPUTextureFormat  surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  // Minimal pipeline resources
  WGPUBuffer             mvpBuffer      = nullptr;
  WGPUBindGroupLayout    bindGroupLayout= nullptr;
  WGPUBindGroup          bindGroup      = nullptr;
  WGPUPipelineLayout     pipelineLayout = nullptr;
  WGPUShaderModule       shader         = nullptr;
  WGPURenderPipeline     pipeline       = nullptr;

  uint32_t width = 0, height = 0;
  bool     initialized = false;

private:
  void createDevice_();
  void createSurface_(const char* canvasSelector);
  void createMVPResources_();
  void createPipeline_();
};
