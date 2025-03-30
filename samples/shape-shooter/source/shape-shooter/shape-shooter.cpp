
/*******************************************************************************

MIT License

Copyright (c) dynamic-static

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*******************************************************************************/

#include "shape-shooter/context.hpp"
#include "shape-shooter/defines.hpp"
#include "shape-shooter/entity-manager.hpp"
#include "shape-shooter/grid.hpp"
#include "shape-shooter/input-manager.hpp"
#include "shape-shooter/player-ship.hpp"
#include "shape-shooter/utilities.hpp"

#include "dynamic-static/text.hpp"
#include "dynamic-static.graphics/coordinate-renderer.hpp"
#include "dynamic-static.graphics/text.hpp"
#include "dynamic-static.graphics/line-renderer.hpp"
#include "dynamic-static.graphics/sprite-renderer.hpp"

#include "stb/stb_image.h"

#include <array>
#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

struct ObjectUniforms
{
    glm::mat4 world{ };
};

struct CameraUniforms
{
    glm::mat4 view{ };
    glm::mat4 projection{ };
};

struct Uniforms
{
    ObjectUniforms object{ };
    CameraUniforms camera{ };
};

VkResult create_camera_resources(const gvk::DescriptorPool& descriptorPool, const gvk::DescriptorSetLayout& descriptorSetLayout, std::pair<gvk::Buffer, gvk::DescriptorSet>& cameraResources)
{
    cameraResources.first.reset();
    cameraResources.second.reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        const auto& device = descriptorSetLayout.get<gvk::Device>();

        // TODO : Documentation
        gvk_result(dst_sample_create_uniform_buffer<Uniforms>(device, &cameraResources.first));

        // TODO : Documentation
        auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
        descriptorSetAllocateInfo.descriptorPool = descriptorPool;
        descriptorSetAllocateInfo.descriptorSetCount = 1;
        descriptorSetAllocateInfo.pSetLayouts = &descriptorSetLayout.get<VkDescriptorSetLayout>();
        gvk_result(gvk::DescriptorSet::allocate(device, &descriptorSetAllocateInfo, &cameraResources.second));

        // TODO : Documentation
        auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
        descriptorBufferInfo.buffer = cameraResources.first;

        // TODO : Documentation
        auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
        writeDescriptorSet.dstSet = cameraResources.second;
        writeDescriptorSet.descriptorCount = 1;
        writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
        device.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);
    } gvk_result_scope_end;
    return gvkResult;
}

void update_camera_uniform_buffer(const gvk::math::Camera& camera, gvk::Buffer uniformBuffer)
{
    CameraUniforms cameraUniforms{ };
    cameraUniforms.view = camera.view();
    cameraUniforms.projection = camera.projection();
    VmaAllocationInfo allocationInfo{ };
    vmaGetAllocationInfo(uniformBuffer.get<gvk::Device>().get<VmaAllocator>(), uniformBuffer.get<VmaAllocation>(), &allocationInfo);
    assert(allocationInfo.pMappedData);
    memcpy(allocationInfo.pMappedData, &cameraUniforms, sizeof(CameraUniforms));
}

int main(int, const char*[])
{
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {

        auto& gameContext = shape_shooter::Context::instance();
        shape_shooter::Context::create({ }, &gameContext);

        // Create a gvk::Context.  This will initialize a VkInstance and VkDevice.
        DstSampleGvkContext gvkContext;
        gvk_result(dst_sample_create_gvk_context("dynamic-static - Shape Shooter", &gvkContext));
        auto gvkDevice = gvkContext.get<gvk::Devices>()[0];
        auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

        // Create a gvk::system::Surface.  This is used to control a system window.
        auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
        systemSurfaceCreateInfo.pTitle = gvkContext.get<gvk::Instance>().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
        systemSurfaceCreateInfo.extent = { 1280, 720 };
        gvk::system::Surface gvkSystemSurface = gvk::nullref;
        dst_vk_result((VkResult)gvk::system::Surface::create(&systemSurfaceCreateInfo, &gvkSystemSurface));

        // Create a gvk::SurfaceKHR
        auto win32SurfaceCreateInfo = gvk::get_default<VkWin32SurfaceCreateInfoKHR>();
        win32SurfaceCreateInfo.hinstance = GetModuleHandle(NULL);
        win32SurfaceCreateInfo.hwnd = gvkSystemSurface.get<gvk::system::Surface::PlatformInfo>().hwnd;
        gvk::SurfaceKHR gvkSurface = VK_NULL_HANDLE;
        dst_vk_result(gvk::SurfaceKHR::create(gvkContext.get<gvk::Instance>(), &win32SurfaceCreateInfo, nullptr, &gvkSurface));

        // Create a gvk::WsiManager.  This is used to manage a connection between the
        //  Vulkan context and the system window.
        auto wsiContextCreateInfo = gvk::get_default<gvk::wsi::Context::CreateInfo>();
        wsiContextCreateInfo.sampleCount = VK_SAMPLE_COUNT_64_BIT;
        wsiContextCreateInfo.depthFormat = VK_FORMAT_D32_SFLOAT;
        wsiContextCreateInfo.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
        wsiContextCreateInfo.queueFamilyIndex = gvkQueue.get<VkDeviceQueueCreateInfo>().queueFamilyIndex;
        gvk::wsi::Context wsiContext = gvk::nullref;
        dst_vk_result(gvk::wsi::Context::create(gvkDevice, gvkSurface, &wsiContextCreateInfo, nullptr, &wsiContext));

        // TODO : Documentation
        std::map<uint32_t, gvk::RenderTarget> renderTargets;
        std::map<uint32_t, shape_shooter::BloomRenderer> bloomRenderers;

        // Create a gvk::gui::Renderer
        gvk::gui::Renderer guiRenderer;
        gvk_result(gvk::gui::Renderer::create(
            gvkContext.get<gvk::Devices>()[0],
            gvk::get_queue_family(gvkContext.get<gvk::Devices>()[0], 0).queues[0],
            gvkContext.get<gvk::CommandBuffers>()[0],
            wsiContext.get<gvk::RenderPass>(),
            nullptr,
            &guiRenderer
        ));

        ///////////////////////////////////////////////////////////////////////////////
        // Bloom
#if 0
        auto wsiContextInfo = wsiContext.get<gvk::wsi::Context::Info>();
        DstSampleRenderTargetCreateInfo renderTargetCreateInfo{ };
        renderTargetCreateInfo.extent = { 1024, 1024 };
        renderTargetCreateInfo.sampleCount = wsiContextInfo.sampleCount;
        renderTargetCreateInfo.colorFormat = wsiContextInfo.surfaceFormat.format;
        renderTargetCreateInfo.depthFormat = wsiContextInfo.depthFormat;
        gvk::RenderTarget renderTarget;
        gvk_result(dst_sample_create_render_target(gvkContext, renderTargetCreateInfo, &renderTarget));
#endif

        // Create the gvk::Sampler that we'll use when we bind the gvk::RenderTarget
        //  color attachment as a shader resource...
        gvk::Sampler sampler;
        gvk_result(gvk::Sampler::create(gvkContext.get<gvk::Devices>()[0], &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &sampler));
        ///////////////////////////////////////////////////////////////////////////////

        // These variables will be controlled via gui widgets
        bool showGui = false;

        ///////////////////////////////////////////////////////////////////////////////
        // Sprites
        gvk::Buffer spriteStagingBuffer;
        std::vector<gvk::ImageView> spriteImages((uint32_t)shape_shooter::Sprite::Count);
        for (uint32_t i = 0; i < (uint32_t)shape_shooter::Sprite::Count; ++i) {
            gvk_result(dst_sample_load_image(gvkContext, shape_shooter::SpriteFilePaths[i], &spriteStagingBuffer, &spriteImages[i]));
        }
        auto spriteColor = gvk::math::Color::White;
        ///////////////////////////////////////////////////////////////////////////////

        shape_shooter::ScoreBoard::create(gvkContext, wsiContext.get<gvk::RenderPass>(), &gameContext.scoreBoard);
        gameContext.pPlayerShip = gameContext.entityManager.create_entity<shape_shooter::PlayerShip>();
        gameContext.particleManager.resize(2048);

        // TODO : Fix FreeCameraController in GVK...
        gvk::math::FreeCameraController cameraController;
        cameraController.verticalLookMin = FLT_MIN;
        cameraController.verticalLookMax = FLT_MAX;
        cameraController.fieldOfViewMin = FLT_MIN;
        cameraController.fieldOfViewMax = FLT_MAX;
        auto f0 = std::numeric_limits<float>::min();
        (void)f0;
        auto f1 = std::numeric_limits<float>::max();
        (void)f1;
        cameraController.set_camera(&gameContext.camera);
        shape_shooter::reset_camera(&gameContext.camera);

        auto cameraDescriptorPoolSize = gvk::get_default<VkDescriptorPoolSize>();
        cameraDescriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cameraDescriptorPoolSize.descriptorCount = 2;
        auto cameraDescriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
        cameraDescriptorPoolCreateInfo.maxSets = 2;
        cameraDescriptorPoolCreateInfo.poolSizeCount = 1;
        cameraDescriptorPoolCreateInfo.pPoolSizes = &cameraDescriptorPoolSize;
        gvk::DescriptorPool cameraDescriptorPool;
        gvk_result(gvk::DescriptorPool::create(gvkContext.get<gvk::Devices>()[0], &cameraDescriptorPoolCreateInfo, nullptr, &cameraDescriptorPool));

        const auto& fontRendererPipeline = gameContext.scoreBoard.get_font_renderer().get_pipeline();
        const auto& fontRendererPipelineLayout = fontRendererPipeline.get<gvk::PipelineLayout>();
        const auto& fontRendererDescriptorSetLayouts = fontRendererPipelineLayout.get<gvk::DescriptorSetLayouts>();
        gvk_result(fontRendererDescriptorSetLayouts.empty() ? VK_ERROR_INITIALIZATION_FAILED : VK_SUCCESS);
        gvk_result(create_camera_resources(cameraDescriptorPool, fontRendererDescriptorSetLayouts[0], gameContext.cameraResources));

        ///////////////////////////////////////////////////////////////////////////////
        // Grid
        shape_shooter::Grid::CreateInfo gridCreateInfo{ };
        gridCreateInfo.extent = { 1920, 1080 };
        gridCreateInfo.cells = { 64, 32 };
        gvk_result(shape_shooter::Grid::create(gvkContext, wsiContext.get<gvk::RenderPass>(), &gridCreateInfo, &shape_shooter::Context::instance().grid));
        ///////////////////////////////////////////////////////////////////////////////

        float spawnInExplosionForce = 5000.0f;
        (void)spawnInExplosionForce;
        dst::RandomNumberGenerator rng;

        // bool updateGameClock = true;

        // gvk::system::Clock clock;
        // auto& clock = shapeShooterContext.programClock;
        while (
            !(gvkSystemSurface.get<gvk::system::Input>().keyboard.down(gvk::system::Key::Escape)) &&
            !(gvkSystemSurface.get<gvk::system::Surface::StatusFlags>() & gvk::system::Surface::CloseRequested)) {
            gvk::system::Surface::update();
            gameContext.audio.update();
            gameContext.programClock.update();
            if (gameContext.gameState != shape_shooter::GameState::Paused) {
                gameContext.gameClock = gameContext.programClock;
            }

            auto frameStart = gvk::system::SteadyClock::now();

            // Update the gvk::math::FreeCameraController...
            auto deltaTime = gameContext.programClock.elapsed<gvk::system::Seconds<float>>();
            const auto& input = gvkSystemSurface.get<gvk::system::Input>();

            // Toggle the gui display with [`]
            if (input.keyboard.pressed(gvk::system::Key::OEM_Tilde)) {
                showGui = !showGui;
            }

            // When ImGui wants mouse/keyboard input, input should be ignored by the scene
            if (!showGui || (!ImGui::GetIO().WantCaptureMouse && !ImGui::GetIO().WantCaptureKeyboard)) {

                // TODO : Documentation
                gameContext.inputManager.update(input);

                // Update camera
                gvk::math::FreeCameraController::UpdateInfo cameraControllerUpdateInfo {
                    /* .deltaTime           = */ deltaTime,
                    /* .moveUp              = */ input.keyboard.down(gvk::system::Key::R),
                    /* .moveDown            = */ input.keyboard.down(gvk::system::Key::Y),
                    /* .moveLeft            = */ input.keyboard.down(gvk::system::Key::F),
                    /* .moveRight           = */ input.keyboard.down(gvk::system::Key::H),
                    /* .moveForward         = */ input.keyboard.down(gvk::system::Key::T),
                    /* .moveBackward        = */ input.keyboard.down(gvk::system::Key::G),
                    /* .moveSpeedMultiplier = */ 32.0f,
                    /* .lookDelta           = */ { input.mouse.position.delta()[0], input.mouse.position.delta()[1] },
                    /* .fieldOfViewDelta    = */ input.mouse.scroll.delta()[1],
                };
                cameraController.lookEnabled = input.mouse.buttons.down(gvk::system::Mouse::Button::Left);
                if (cameraController.lookEnabled) {
                    gvkSystemSurface.set(gvk::system::Surface::CursorMode::Hidden);
                } else {
                    gvkSystemSurface.set(gvk::system::Surface::CursorMode::Visible);
                }
                cameraController.update(cameraControllerUpdateInfo);

                // Switch on GameState for input logic
                switch (gameContext.gameState) {
                case shape_shooter::GameState::Attract: {
                    if (input.keyboard.pressed(gvk::system::Key::SpaceBar)) {
                        gameContext.gameState = shape_shooter::GameState::Playing;
                    }
                } break;
                case shape_shooter::GameState::Playing: {
                    if (input.keyboard.pressed(gvk::system::Key::SpaceBar)) {
                        gameContext.gameState = shape_shooter::GameState::Paused;
                        gameContext.gameClock = { };
                    }
                } break;
                case shape_shooter::GameState::Paused: {
                    if (input.keyboard.pressed(gvk::system::Key::SpaceBar)) {
                        gameContext.gameState = shape_shooter::GameState::Playing;
                    }
                } break;
                case shape_shooter::GameState::GameOver: {
                    if (input.keyboard.pressed(gvk::system::Key::SpaceBar)) {
                        gameContext.gameState = shape_shooter::GameState::Attract;
                        gameContext.scoreBoard.reset_score();
                    }
                } break;
                default: {
                    assert(false);
                } break;
                }
            }

            // Switch on GameState for update logic
            switch (gameContext.gameState) {
            case shape_shooter::GameState::Attract: {
            } break;
            case shape_shooter::GameState::Playing: {
            } break;
            case shape_shooter::GameState::Paused: {
            } break;
            case shape_shooter::GameState::GameOver: {
            } break;
            default: {
                assert(false);
            } break;
            }

#if 0
            auto mouseAimPoint = shape_shooter::Context::instance().inputManager.get_mouse_aim_point();
            if (input.keyboard.pressed(gvk::system::Key::C)) {
                // std::cout << glm::to_string(shapeShooterContext.gameCamera.transform.translation) << std::endl; // 0.35, 996.331, -16.84
                // shape_shooter::Context::instance().grid.apply_directed_force({ 0, 0, 5000 }, { }, 50);

                // apply_directed_force() is used just once, for the ship spawn in
                shape_shooter::Context::instance().grid.apply_directed_force({ 0, -spawnInExplosionForce, 0 }, mouseAimPoint, 50);
            }
            if (input.keyboard.pressed(gvk::system::Key::V)) {
                // apply_directed_force() is used just once, for the black hole
                shape_shooter::Context::instance().grid.apply_implosive_force((float)(std::sin(M_PI_2) * 10.0 + 20.0), mouseAimPoint, 200);
            }
            if (input.keyboard.pressed(gvk::system::Key::B)) {
                // apply_directed_force() is used just once, for the bullets
                shape_shooter::Context::instance().grid.apply_explosive_force(8, mouseAimPoint, 80);
            }
#endif

            update_camera_uniform_buffer(gameContext.camera, gameContext.cameraResources.first);

            ///////////////////////////////////////////////////////////////////////////////
            gameContext.scoreBoard.update();
            gameContext.playerStatus.update(gameContext);
            gameContext.enemySpawner.update(gameContext);
            gameContext.entityManager.update(gameContext);
            gameContext.particleManager.update();
            gameContext.grid.update(gameContext.gameClock.elapsed<gvk::system::Seconds<float>>());
            ///////////////////////////////////////////////////////////////////////////////

            gvk::wsi::AcquiredImageInfo acquiredImageInfo{ };
            gvk::RenderTarget acquiredImageRenderTarget = VK_NULL_HANDLE;
            auto wsiStatus = wsiContext.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &acquiredImageInfo, &acquiredImageRenderTarget);
            if (wsiStatus == VK_SUCCESS || wsiStatus == VK_SUBOPTIMAL_KHR) {
                const auto& device = gvkContext.get<gvk::Devices>()[0];
                auto extent = wsiContext.get<gvk::SwapchainKHR>().get<VkSwapchainCreateInfoKHR>().imageExtent;
                gameContext.camera.set_aspect_ratio(extent.width, extent.height);
                gameContext.renderExtent = { extent.width, extent.height };

                gvk_result(dst_sample_debug_marker_set_render_target_name(acquiredImageRenderTarget, "gvk::SwapchainKHR[" + std::to_string(acquiredImageInfo.index) + "]"));

                // TODO : Documentation
                auto& renderTarget = renderTargets[acquiredImageInfo.index];
                auto renderTargetCreateInfo = renderTarget ? renderTarget.get<VkFramebufferCreateInfo>() : gvk::get_default<VkFramebufferCreateInfo>();
                if (renderTargetCreateInfo.width != extent.width || renderTargetCreateInfo.height != extent.height) {
                    auto dstSampleRenderTargetCreateInfo = gvk::get_default<DstSampleRenderTargetCreateInfo>();
                    dstSampleRenderTargetCreateInfo.extent = extent;
                    dstSampleRenderTargetCreateInfo.colorFormat = wsiContext.get<gvk::wsi::Context::Info>().surfaceFormat.format;
                    dstSampleRenderTargetCreateInfo.depthFormat = wsiContext.get<gvk::wsi::Context::Info>().depthFormat;
                    dstSampleRenderTargetCreateInfo.sampleCount = VK_SAMPLE_COUNT_64_BIT;
                    gvk_result(dst_sample_create_render_target(gvkContext, dstSampleRenderTargetCreateInfo, &renderTarget));
                    gvk_result(dst_sample_debug_marker_set_render_target_name(renderTarget, "Scene[" + std::to_string(acquiredImageInfo.index) + "]"));
                }

                // TODO : Documentation
                auto bloomRendererItr = bloomRenderers.find(acquiredImageInfo.index);
                if (bloomRendererItr == bloomRenderers.end()) {
                    shape_shooter::BloomRenderer::CreateInfo bloomRendererCreateInfo{ };
                    bloomRendererCreateInfo.format = wsiContext.get<gvk::wsi::Context::Info>().surfaceFormat.format;
                    bloomRendererCreateInfo.renderPass = wsiContext.get<gvk::RenderPass>();
                    bloomRendererItr = bloomRenderers.insert({ acquiredImageInfo.index, { } }).first;
                    gvk_result(shape_shooter::BloomRenderer::create(gvkContext, &bloomRendererCreateInfo, &bloomRendererItr->second));
                }
                auto& bloomRenderer = bloomRendererItr->second;
                (void)bloomRenderer;

                // TODO : Documentation
                auto spriteRendererItr = gameContext.spriteRenderers.find(acquiredImageInfo.index);
                if (spriteRendererItr == gameContext.spriteRenderers.end()) {
                    dst::gfx::SpriteRenderer::CreateInfo spriteRendererCreateInfo{ };
                    spriteRendererCreateInfo.renderPass = wsiContext.get<gvk::RenderPass>();
                    spriteRendererCreateInfo.imageCount = (uint32_t)spriteImages.size();
                    spriteRendererCreateInfo.pImages = spriteImages.data();
                    spriteRendererItr = gameContext.spriteRenderers.insert({ acquiredImageInfo.index, { } }).first;
                    gvk_result(dst::gfx::SpriteRenderer::create(gvkContext, spriteRendererCreateInfo, &spriteRendererItr->second));
                }
                auto& spriteRenderer = spriteRendererItr->second;
                spriteRenderer.begin_sprite_batch();
                gameContext.particleManager.draw(spriteRenderer);
                gameContext.entityManager.draw(spriteRenderer);
                spriteRenderer.end_sprite_batch();

                // If the gvk::gui::Renderer is enabled, update values based on gui interaction
                if (showGui) {
                    // Update the gvk::system::Surface::CursorMode mode based on gui interaction
                    auto imguiCursor = ImGui::GetMouseCursor();
                    if (imguiCursor == ImGuiMouseCursor_None || ImGui::GetIO().MouseDrawCursor) {
                        gvkSystemSurface.set(gvk::system::Surface::CursorMode::Hidden);
                    } else {
                        switch (imguiCursor) {
                        case ImGuiMouseCursor_Arrow: gvkSystemSurface.set(gvk::system::Surface::CursorType::Arrow); break;
                        case ImGuiMouseCursor_TextInput: gvkSystemSurface.set(gvk::system::Surface::CursorType::IBeam); break;
                        case ImGuiMouseCursor_Hand: gvkSystemSurface.set(gvk::system::Surface::CursorType::Hand); break;
                        case ImGuiMouseCursor_ResizeNS: gvkSystemSurface.set(gvk::system::Surface::CursorType::ResizeNS); break;
                        case ImGuiMouseCursor_ResizeEW: gvkSystemSurface.set(gvk::system::Surface::CursorType::ResizeEW); break;
                        case ImGuiMouseCursor_ResizeAll: gvkSystemSurface.set(gvk::system::Surface::CursorType::ResizeAll); break;
                        case ImGuiMouseCursor_ResizeNESW: gvkSystemSurface.set(gvk::system::Surface::CursorType::ResizeNESW); break;
                        case ImGuiMouseCursor_ResizeNWSE: gvkSystemSurface.set(gvk::system::Surface::CursorType::ResizeNWSE); break;
                        case ImGuiMouseCursor_NotAllowed: gvkSystemSurface.set(gvk::system::Surface::CursorType::NotAllowed); break;
                        default: break;
                        }
                    }
                    if (gvkSystemSurface.get<gvk::system::Surface::StatusFlags>() & gvk::system::Surface::GainedFocus) {
                        ImGui::GetIO().AddFocusEvent(true);
                    }
                    if (gvkSystemSurface.get<gvk::system::Surface::StatusFlags>() & gvk::system::Surface::LostFocus) {
                        ImGui::GetIO().AddFocusEvent(false);
                    }

                    // Prepare a gvk::gui::Renderer::BeginInfo
                    const auto& textStream = gvkSystemSurface.get<gvk::system::Surface::TextStream>();
                    auto guiRendererBeginInfo = gvk::get_default<gvk::gui::Renderer::BeginInfo>();
                    guiRendererBeginInfo.deltaTime = deltaTime;
                    guiRendererBeginInfo.extent = { (float)extent.width, (float)extent.height };
                    guiRendererBeginInfo.pInput = &input;
                    guiRendererBeginInfo.textStreamCodePointCount = (uint32_t)textStream.size();
                    guiRendererBeginInfo.pTextStreamCodePoints = !textStream.empty() ? textStream.data() : nullptr;

                    // Call guiRenderer.begin_gui().  Note that all ImGui widgets must be handled
                    //  between calls to begin_gui()/end_gui(), which are called once per frame
                    guiRenderer.begin_gui(guiRendererBeginInfo);
                    shape_shooter::on_gui(gameContext);
                    bloomRenderer.on_gui();
                    guiRenderer.end_gui(acquiredImageInfo.index);
                }

                gvk_result(vkBeginCommandBuffer(acquiredImageInfo.commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>()));

#if 0
                // Begin the gvk::RenderPass that renders into the gvk::WsiManager...
                auto renderPassBeginInfo = acquiredImageRenderTarget.get<VkRenderPassBeginInfo>();
                vkCmdBeginRenderPass(acquiredImageInfo.commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor{ { }, renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(acquiredImageInfo.commandBuffer, 0, 1, &scissor);
                    VkViewport viewport{ 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
                    vkCmdSetViewport(acquiredImageInfo.commandBuffer, 0, 1, &viewport);

                    // Bind the gvk::math::Camera uniform gvk::Buffer and the floor resources then
                    //  issue a draw call for the floor.  Then bind the floating cube resources...
                    //  we can leave the gvk::math::Camera uniform gvk::Buffer bound and update the
                    //  gvk::Pipeline and gvk::DescriptorSet at index 1 without distrubing the
                    //  gvk::DescriptorSet at index 0...then issue a draw call for the floating
                    //  cube...
                    auto pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;

                    // Grid
                    shape_shooter::Context::instance().grid.record_draw_cmds(acquiredImageInfo.commandBuffer, gameContext.camera, gameContext.renderExtent);

                    // TODO : Draw sprites additively w/depth, then render grid (maybe?)
                    // Sprites
                    auto spriteCamera = gameContext.camera;
                    // spriteCamera.projectionMode = gvk::math::Camera::ProjectionMode::Orthographic;
                    //spriteCamera.fieldOfView = viewport.width;
                    spriteRenderer.record_draw_cmds(acquiredImageInfo.commandBuffer, spriteCamera);

                    // ScoreBoard
                    const auto& gameCameraDescriptorSet = gameContext.cameraResources.second;
                    vkCmdBindDescriptorSets(acquiredImageInfo.commandBuffer, pipelineBindPoint, fontRendererPipelineLayout, 0, 1, &gameCameraDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
                    shape_shooter::Context::instance().scoreBoard.record_draw_cmds(acquiredImageInfo.commandBuffer);

                    // If the gvk::gui::Renderer is enabled, record cmds to render it
                    if (showGui) {
                        guiRenderer.record_cmds(acquiredImageInfo.commandBuffer, acquiredImageInfo.index);
                    }
                }
                vkCmdEndRenderPass(acquiredImageInfo.commandBuffer);
#else
                // TODO : Documentation
                auto renderPassBeginInfo = renderTarget.get<VkRenderPassBeginInfo>();
                vkCmdBeginRenderPass(acquiredImageInfo.commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor{ { }, renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(acquiredImageInfo.commandBuffer, 0, 1, &scissor);
                    VkViewport viewport{ 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
                    vkCmdSetViewport(acquiredImageInfo.commandBuffer, 0, 1, &viewport);

                    // Grid
                    shape_shooter::Context::instance().grid.record_draw_cmds(acquiredImageInfo.commandBuffer, gameContext.camera, gameContext.renderExtent);

                    // TODO : Draw sprites additively w/depth, then render grid (maybe?)
                    // Sprites
                    auto spriteCamera = gameContext.camera;
                    // spriteCamera.projectionMode = gvk::math::Camera::ProjectionMode::Orthographic;
                    //spriteCamera.fieldOfView = viewport.width;
                    spriteRenderer.record_draw_cmds(acquiredImageInfo.commandBuffer, spriteCamera);
                }
                vkCmdEndRenderPass(acquiredImageInfo.commandBuffer);
#if 1
                // TODO : Documentation
                bloomRenderer.record_cmds(gvkContext, acquiredImageInfo.commandBuffer, wsiContext.get<gvk::wsi::Context::Info>().surfaceFormat.format, renderTarget);
#endif
                // TODO : Documentation
                renderPassBeginInfo = acquiredImageRenderTarget.get<VkRenderPassBeginInfo>();
                vkCmdBeginRenderPass(acquiredImageInfo.commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor{ { }, renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(acquiredImageInfo.commandBuffer, 0, 1, &scissor);
                    VkViewport viewport{ 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
                    vkCmdSetViewport(acquiredImageInfo.commandBuffer, 0, 1, &viewport);
                    auto pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;

                    // Bloom result
                    bloomRenderer.draw_render_target(acquiredImageInfo.commandBuffer);

                    // ScoreBoard
                    const auto& gameCameraDescriptorSet = gameContext.cameraResources.second;
                    vkCmdBindDescriptorSets(acquiredImageInfo.commandBuffer, pipelineBindPoint, fontRendererPipelineLayout, 0, 1, &gameCameraDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
                    shape_shooter::Context::instance().scoreBoard.record_draw_cmds(acquiredImageInfo.commandBuffer);

                    // If the gvk::gui::Renderer is enabled, record cmds to render it
                    if (showGui) {
                        guiRenderer.record_cmds(acquiredImageInfo.commandBuffer, acquiredImageInfo.index);
                    }
                }
                vkCmdEndRenderPass(acquiredImageInfo.commandBuffer);
#endif

                // TODO : Documentation
                // TODO : GVK should provide a prepared VkImageMemoryBarrier array
                auto attachmentCount = renderTarget.get<gvk::Framebuffer>().get<gvk::RenderPass>().get<VkRenderPassCreateInfo2>().attachmentCount;
                for (size_t attachment_i = 0; attachment_i < attachmentCount; ++attachment_i) {
                    auto imageMemoryBarrier = renderTarget.get<VkImageMemoryBarrier>((uint32_t)attachment_i);
                    if (imageMemoryBarrier.oldLayout != imageMemoryBarrier.newLayout) {
                        vkCmdPipelineBarrier(
                            acquiredImageInfo.commandBuffer,
                            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                            0,
                            0, nullptr,
                            0, nullptr,
                            1, &imageMemoryBarrier
                        );
                    }
                }

                gvk_result(vkEndCommandBuffer(acquiredImageInfo.commandBuffer));

                const auto& queue = gvk::get_queue_family(device, 0).queues[0];
                gvk_result(vkQueueSubmit(queue, 1, &wsiContext.get<VkSubmitInfo>(acquiredImageInfo), acquiredImageInfo.fence));

                wsiStatus = wsiContext.queue_present(queue, &acquiredImageInfo);
                gvk_result((wsiStatus == VK_SUBOPTIMAL_KHR || wsiStatus == VK_ERROR_OUT_OF_DATE_KHR) ? VK_SUCCESS : wsiStatus);
            }

            static const gvk::system::Milliseconds<> FrameDuration(shape_shooter::OneOverSixty * 1000.0f);
            auto frameEnd = gvk::system::SteadyClock::now();
            auto frameTime = frameEnd - frameStart;
            if (frameTime < FrameDuration) {
                std::this_thread::sleep_for(gvk::system::duration_cast<gvk::system::Milliseconds<>>(FrameDuration - frameTime));
            }

            static uint64_t sFrameCount;
            ++sFrameCount;
            static decltype(gameContext.programClock.elapsed<gvk::system::Seconds<>>()) sFpsTimer;
            sFpsTimer += gameContext.programClock.elapsed<gvk::system::Seconds<>>();
            if (1.0f <= sFpsTimer) {
                // std::cout << sFrameCount << " / " << sFpsTimer << std::endl;
                sFrameCount = 0;
                sFpsTimer = 0;
            }
        }
        gvk_result(vkDeviceWaitIdle(gvkContext.get<gvk::Devices>()[0]));
    } gvk_result_scope_end;
    if (gvkResult) {
        std::cerr << gvk::to_string(gvkResult) << std::endl;
    }
    return (int)gvkResult;
}
