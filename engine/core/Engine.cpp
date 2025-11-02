#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "render/WebGPUContext.h"
#include "render/RenderMesh.h"    // for render::g_data
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"
#include "math/MiniMath.h"

using render::g_data;  // global GPU context data

Engine::Engine() = default;
Engine::~Engine() = default;

void Engine::Init()
{
#if FE_WEBGPU
    // Initialize WebGPU context (device, queue, surface, etc.)
    render::WebGPUContext::Init();
#endif

    scene = std::make_unique<Scene>();
    game  = std::make_unique<Game>();

    // Add a grid floor for visual reference
    auto grid = std::make_shared<GameObject>();
    grid->mesh = std::make_shared<RenderMesh>();
    grid->mesh->UploadCPU(render::g_data.device, geom::GridPlane(20, 20, 1.0f));
    scene->AddObject(grid);

    // Add a simple cross marker at origin
    auto marker = std::make_shared<GameObject>();
    marker->mesh = std::make_shared<RenderMesh>();
    marker->mesh->UploadCPU(render::g_data.device, geom::MarkerCross());
    scene->AddObject(marker);

    Time::Init();
}

void Engine::Update()
{
    Time::Update();
    float dt = Time::DeltaTime();

    if (game)
        game->Update(dt);

    if (scene)
        scene->Update(dt);
}

void Engine::Render()
{
#if FE_WEBGPU
    auto device = render::g_data.device;
    auto queue  = render::g_data.queue;
    auto ctx    = render::g_data.context;

    if (!device || !queue || !ctx)
        return;

    // Begin WebGPU frame
    WGPUSurfaceTexture surfaceTex;
    wgpuSurfaceGetCurrentTexture(ctx.surface, &surfaceTex);

    WGPUTextureView view = wgpuTextureCreateView(surfaceTex.texture, nullptr);

    WGPUCommandEncoderDescriptor encDesc{};
    WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);

    // (You’ll add rendering passes here later)
    // For now, just clear the screen to gray.
    WGPURenderPassColorAttachment colorAttach{};
    colorAttach.view = view;
    colorAttach.clearValue = {0.3f, 0.3f, 0.3f, 1.0f};
    colorAttach.loadOp = WGPULoadOp_Clear;
    colorAttach.storeOp = WGPUStoreOp_Store;

    WGPURenderPassDescriptor passDesc{};
    passDesc.colorAttachmentCount = 1;
    passDesc.colorAttachments = &colorAttach;

    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
    wgpuRenderPassEncoderEnd(pass);

    WGPUCommandBufferDescriptor cmdBufDesc{};
    WGPUCommandBuffer cmdBuf = wgpuCommandEncoderFinish(encoder, &cmdBufDesc);
    wgpuQueueSubmit(queue, 1, &cmdBuf);

    wgpuTextureViewRelease(view);
    wgpuCommandBufferRelease(cmdBuf);
    wgpuCommandEncoderRelease(encoder);
#endif
}

void Engine::Shutdown()
{
    scene.reset();
    game.reset();

#if FE_WEBGPU
    render::WebGPUContext::Shutdown();
#endif
}
