
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
        // Create a gvk::Context.  This will initialize a VkInstance and VkDevice.
        DstSampleGvkContext gvkContext;
        gvk_result(dst_sample_create_gvk_context("dynamic-static - Shape Shooter", &gvkContext));
        auto gvkDevice = gvkContext.get_devices()[0];
        auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

        // Create a gvk::system::Surface.  This is used to control a system window.
        auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
        systemSurfaceCreateInfo.pTitle = gvkContext.get_instance().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
        systemSurfaceCreateInfo.extent = { 1280, 720 };
        gvk::system::Surface systemSurface;
        auto success = gvk::system::Surface::create(&systemSurfaceCreateInfo, &systemSurface);
        (void)success;
        assert(success);

        // Create a gvk::WsiManager.  This is used to manage a connection between the
        //  Vulkan context and the system window.
        auto wsiManagerCreateInfo = gvk::get_default<gvk::WsiManager::CreateInfo>();
        auto win32SurfaceCreateInfo = gvk::get_default<VkWin32SurfaceCreateInfoKHR>();
        win32SurfaceCreateInfo.hinstance = GetModuleHandle(NULL);
        win32SurfaceCreateInfo.hwnd = (HWND)systemSurface.get_hwnd();
        wsiManagerCreateInfo.pWin32SurfaceCreateInfoKHR = &win32SurfaceCreateInfo;
        wsiManagerCreateInfo.sampleCount = VK_SAMPLE_COUNT_64_BIT;
        wsiManagerCreateInfo.depthFormat = VK_FORMAT_D32_SFLOAT;
        wsiManagerCreateInfo.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
        wsiManagerCreateInfo.queueFamilyIndex = gvkQueue.get<VkDeviceQueueCreateInfo>().queueFamilyIndex;
        gvk::WsiManager wsiManager;
        gvk_result(gvk::WsiManager::create(gvkDevice, &wsiManagerCreateInfo, nullptr, &wsiManager));

        // Create a gvk::gui::Renderer
        gvk::gui::Renderer guiRenderer;
        gvk_result(gvk::gui::Renderer::create(
            gvkContext.get_devices()[0],
            gvk::get_queue_family(gvkContext.get_devices()[0], 0).queues[0],
            gvkContext.get_command_buffers()[0],
            wsiManager.get_render_pass(),
            nullptr,
            &guiRenderer
        ));

        // Create a gvk::RenderTarget.  We're going to want to be able to render to
        //  this gvk::RenderTarget and the gvk::WsiManager gvk::RenderTarget objects
        //  using the same gvk::Pipeline objects so the gvk::RenderPass objects need to
        //  be compatible...
        DstSampleRenderTargetCreateInfo renderTargetCreateInfo{ };
        renderTargetCreateInfo.extent = { 1024, 1024 };
        renderTargetCreateInfo.sampleCount = wsiManager.get_sample_count();
        renderTargetCreateInfo.colorFormat = wsiManager.get_color_format();
        renderTargetCreateInfo.depthFormat = wsiManager.get_depth_format();
        gvk::RenderTarget renderTarget;
        gvk_result(dst_sample_create_render_target(gvkContext, renderTargetCreateInfo, &renderTarget));

        // Create the gvk::Sampler that we'll use when we bind the gvk::RenderTarget
        //  color attachment as a shader resource...
        gvk::Sampler sampler;
        gvk_result(gvk::Sampler::create(gvkContext.get_devices()[0], &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &sampler));

        // These variables will be controlled via gui widgets
        bool showGui = false;
        int lookType = 0;

        ///////////////////////////////////////////////////////////////////////////////
        // CoordinateRenderer
        dst::gfx::CoordinateRenderer::CreateInfo coordinateRendererCreateInfo{ };
        coordinateRendererCreateInfo.renderPass = wsiManager.get_render_pass();
        coordinateRendererCreateInfo.pTtfFilePath = "C:\\Windows\\Fonts\\bauhs93.ttf";
        dst::gfx::CoordinateRenderer coordinateRenderer;
        gvk_result(dst::gfx::CoordinateRenderer::create(gvkContext, coordinateRendererCreateInfo, &coordinateRenderer));
        ///////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////
        // Sprites
        gvk::Buffer spriteStagingBuffer;
        std::vector<gvk::ImageView> spriteImages((uint32_t)shape_shooter::Sprite::Count);
        for (uint32_t i = 0; i < (uint32_t)shape_shooter::Sprite::Count; ++i) {
            gvk_result(dst_sample_load_image(gvkContext, shape_shooter::SpriteFilePaths[i], &spriteStagingBuffer, &spriteImages[i]));
        }
        dst::gfx::SpriteRenderer::CreateInfo spriteRendererCreateInfo { };
        spriteRendererCreateInfo.renderPass = wsiManager.get_render_pass();
        spriteRendererCreateInfo.imageCount = (uint32_t)spriteImages.size();
        spriteRendererCreateInfo.pImages = spriteImages.data();
        gvk_result(dst::gfx::SpriteRenderer::create(gvkContext, spriteRendererCreateInfo, &shape_shooter::Context::instance().spriteRenderer));
        auto spriteColor = gvk::math::Color::White;
        ///////////////////////////////////////////////////////////////////////////////

        auto& shapeShooterContext = shape_shooter::Context::instance();
        shape_shooter::ScoreBoard::create(gvkContext, wsiManager.get_render_pass(), &shapeShooterContext.scoreBoard);
        shapeShooterContext.pPlayerShip = shapeShooterContext.entityManager.create_entity<shape_shooter::PlayerShip>();
        shapeShooterContext.particleManager.resize(2048);

#if 0
        shapeShooterContext.scoreBoardCamera.transform.translation = { 12.1287f, 32.5891f, -438.474f };
        shapeShooterContext.scoreBoardCamera.transform.rotation = glm::quat(glm::vec3{ 16.0f, -45.0f, -2.0f } * glm::pi<float>() / 180.0f);
#endif

        shapeShooterContext.gameCamera.farPlane = 1000.0f;
        shapeShooterContext.gameCamera.transform.translation = { 0, 2, -7 };
        gvk::math::FreeCameraController cameraController;
        cameraController.set_camera(&shapeShooterContext.gameCamera);

        auto cameraDescriptorPoolSize = gvk::get_default<VkDescriptorPoolSize>();
        cameraDescriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cameraDescriptorPoolSize.descriptorCount = 2;
        auto cameraDescriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
        cameraDescriptorPoolCreateInfo.maxSets = 2;
        cameraDescriptorPoolCreateInfo.poolSizeCount = 1;
        cameraDescriptorPoolCreateInfo.pPoolSizes = &cameraDescriptorPoolSize;
        gvk::DescriptorPool cameraDescriptorPool;
        gvk_result(gvk::DescriptorPool::create(gvkContext.get_devices()[0], &cameraDescriptorPoolCreateInfo, nullptr, &cameraDescriptorPool));

        const auto& fontRendererPipeline = shapeShooterContext.scoreBoard.get_font_renderer().get_pipeline();
        const auto& fontRendererPipelineLayout = fontRendererPipeline.get<gvk::PipelineLayout>();
        const auto& fontRendererDescriptorSetLayouts = fontRendererPipelineLayout.get<gvk::DescriptorSetLayouts>();
        gvk_result(fontRendererDescriptorSetLayouts.empty() ? VK_ERROR_INITIALIZATION_FAILED : VK_SUCCESS);
        gvk_result(create_camera_resources(cameraDescriptorPool, fontRendererDescriptorSetLayouts[0], shapeShooterContext.gameCameraResources));
        gvk_result(create_camera_resources(cameraDescriptorPool, fontRendererDescriptorSetLayouts[0], shapeShooterContext.scoreBoardCameraResources));

        ///////////////////////////////////////////////////////////////////////////////
        // Grid
        shape_shooter::Grid::CreateInfo gridCreateInfo{ };
        gridCreateInfo.extent = { 1920, 1080 };
        gridCreateInfo.cells = { 64, 32 };
        gvk_result(shape_shooter::Grid::create(gvkContext, wsiManager.get_render_pass(), &gridCreateInfo, &shape_shooter::Context::instance().grid));
        ///////////////////////////////////////////////////////////////////////////////

        // gvk::system::Clock clock;
        auto& clock = shapeShooterContext.clock;
        while (
            !(systemSurface.get_input().keyboard.down(gvk::system::Key::Escape)) &&
            !(systemSurface.get_status() & gvk::system::Surface::CloseRequested)) {
            gvk::system::Surface::update();
            clock.update();

            // Update the gvk::math::FreeCameraController...
            auto deltaTime = clock.elapsed<gvk::system::Seconds<float>>();
            const auto& input = systemSurface.get_input();

            // Toggle the gui display with [`]
            if (input.keyboard.pressed(gvk::system::Key::OEM_Tilde)) {
                showGui = !showGui;
            }

            // When ImGui wants mouse/keyboard input, input should be ignored by the scene
            if (!ImGui::GetIO().WantCaptureMouse && !ImGui::GetIO().WantCaptureKeyboard) {
                gvk::math::FreeCameraController::UpdateInfo cameraControllerUpdateInfo {
                    /* .deltaTime           = */ deltaTime,
                    /* .moveUp              = */ input.keyboard.down(gvk::system::Key::Q),
                    /* .moveDown            = */ input.keyboard.down(gvk::system::Key::E),
                    /* .moveLeft            = */ input.keyboard.down(gvk::system::Key::A),
                    /* .moveRight           = */ input.keyboard.down(gvk::system::Key::D),
                    /* .moveForward         = */ input.keyboard.down(gvk::system::Key::W),
                    /* .moveBackward        = */ input.keyboard.down(gvk::system::Key::S),
                    /* .moveSpeedMultiplier = */ input.keyboard.down(gvk::system::Key::LeftShift) ? 24.0f : 1.0f,
                    /* .lookDelta           = */ { input.mouse.position.delta()[0], input.mouse.position.delta()[1] },
                    /* .fieldOfViewDelta    = */ input.mouse.scroll.delta()[1],
                };
                cameraController.lookEnabled = input.mouse.buttons.down(gvk::system::Mouse::Button::Left);
                if (cameraController.lookEnabled) {
                    systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Hidden);
                } else {
                    systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Visible);
                }
                if (input.mouse.buttons.pressed(gvk::system::Mouse::Button::Right)) {
                    shapeShooterContext.gameCamera.fieldOfView = 60.0f;
                }
                cameraController.update(cameraControllerUpdateInfo);
            }

            if (input.keyboard.pressed(gvk::system::Key::Backspace)) {
                lookType = lookType ? 0 : 1;
                if (lookType) {
                    shapeShooterContext.gameCamera.transform.translation = { 0, 965, 0 };
                    shapeShooterContext.gameCamera.transform.rotation = glm::normalize(glm::angleAxis(glm::radians(90.0f), glm::vec3{ 1, 0, 0 }));
                }
            }

            static bool lockScoreBoardCamera = false;
            if (input.keyboard.pressed(gvk::system::Key::L)) {
                lockScoreBoardCamera = !lockScoreBoardCamera;
#if 0
                const auto& t = shapeShooterContext.gameCamera.transform;
                const auto& p = t.translation;
                const auto& r = glm::eulerAngles(t.rotation) * 180.0f / glm::pi<float>();
                const auto& s = t.scale;
                std::cout << "================================================================================" << std::endl;
                std::cout << "translation : {" << p.x << "," << p.y << "," << p.z << "}" << std::endl;
                std::cout << "rotation    : {" << r.x << "," << r.y << "," << r.z << "}" << std::endl;
                std::cout << "scale       : {" << s.x << "," << s.y << "," << s.z << "}" << std::endl;
                std::cout << "================================================================================" << std::endl;
#endif
            }
            if (!lockScoreBoardCamera) {
                shapeShooterContext.scoreBoardCamera = shapeShooterContext.gameCamera;
            }
            update_camera_uniform_buffer(shapeShooterContext.gameCamera, shapeShooterContext.gameCameraResources.first);
            update_camera_uniform_buffer(shapeShooterContext.scoreBoardCamera, shapeShooterContext.scoreBoardCameraResources.first);

            ///////////////////////////////////////////////////////////////////////////////
            // CoordinateRenderer
            coordinateRenderer.update();
            ///////////////////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////////////////
            // shape_shooter::Context
            shapeShooterContext.inputManager.update(input);
            shapeShooterContext.entityManager.update();
            shapeShooterContext.enemySpawner.update();
            shapeShooterContext.scoreBoard.update();
            shapeShooterContext.particleManager.update();
            shapeShooterContext.spriteRenderer.begin_sprite_batch();

            // SpriteSortMode.Texture
            shapeShooterContext.entityManager.draw();
            // SpriteSortMode.Deferred
            shapeShooterContext.particleManager.draw();
            shapeShooterContext.spriteRenderer.end_sprite_batch();
            ///////////////////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////////////////
            // Grid
            shape_shooter::Context::instance().grid.update(deltaTime);
            ///////////////////////////////////////////////////////////////////////////////

            wsiManager.update();
            auto swapchain = wsiManager.get_swapchain();
            if (swapchain) {
                uint32_t imageIndex = 0;
                auto vkResult = wsiManager.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &imageIndex);
                gvk_result((vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR) ? VK_SUCCESS : vkResult);

                auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
                shapeShooterContext.gameCamera.set_aspect_ratio(extent.width, extent.height);
                shapeShooterContext.renderExtent = { extent.width, extent.height };

                // Get VkFences from the WsiManager.  The gvk::gui::Renderer will wait on these
                //  VkFences to ensure that it doesn't destroy any resources that are still in
                //  use by the WsiManager
                const auto& vkFences = wsiManager.get_vk_fences();

                // If the gvk::gui::Renderer is enabled, update values based on gui interaction
                if (showGui) {
                    // Update the gvk::system::Surface::CursorMode mode based on gui interaction
                    auto imguiCursor = ImGui::GetMouseCursor();
                    if (imguiCursor == ImGuiMouseCursor_None || ImGui::GetIO().MouseDrawCursor) {
                        systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Hidden);
                    } else {
                        switch (imguiCursor) {
                        case ImGuiMouseCursor_Arrow: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::Arrow); break;
                        case ImGuiMouseCursor_TextInput: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::IBeam); break;
                        case ImGuiMouseCursor_Hand: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::Hand); break;
                        case ImGuiMouseCursor_ResizeNS: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeNS); break;
                        case ImGuiMouseCursor_ResizeEW: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeEW); break;
                        case ImGuiMouseCursor_ResizeAll: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeAll); break;
                        case ImGuiMouseCursor_ResizeNESW: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeNESW); break;
                        case ImGuiMouseCursor_ResizeNWSE: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeNWSE); break;
                        case ImGuiMouseCursor_NotAllowed: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::NotAllowed); break;
                        default: break;
                        }
                    }
                    if (systemSurface.get_status() & gvk::system::Surface::GainedFocus) {
                        ImGui::GetIO().AddFocusEvent(true);
                    }
                    if (systemSurface.get_status() & gvk::system::Surface::LostFocus) {
                        ImGui::GetIO().AddFocusEvent(false);
                    }

                    // Prepare a gvk::gui::Renderer::BeginInfo
                    const auto& textStream = systemSurface.get_text_stream();
                    auto guiRendererBeginInfo = gvk::get_default<gvk::gui::Renderer::BeginInfo>();
                    guiRendererBeginInfo.deltaTime = deltaTime;
                    guiRendererBeginInfo.extent = { (float)extent.width, (float)extent.height };
                    guiRendererBeginInfo.pInput = &input;
                    guiRendererBeginInfo.textStreamCodePointCount = (uint32_t)textStream.size();
                    guiRendererBeginInfo.pTextStreamCodePoints = !textStream.empty() ? textStream.data() : nullptr;

                    // Call guiRenderer.begin_gui().  Note that all ImGui widgets must be handled
                    //  between calls to begin_gui()/end_gui()
                    guiRenderer.begin_gui(guiRendererBeginInfo);
                    ImGui::ShowDemoWindow();
                    guiRenderer.end_gui((uint32_t)vkFences.size(), !vkFences.empty() ? vkFences.data() : nullptr);
                }

                const auto& device = gvkContext.get_devices()[0];
                gvk_result(vkWaitForFences(device, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX));
                gvk_result(vkResetFences(device, 1, &vkFences[imageIndex]));

                const auto& commandBuffer = wsiManager.get_command_buffers()[imageIndex];
                gvk_result(vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>()));

                // Begin the gvk::RenderPass that renders into the gvk::WsiManager...
                auto renderPassBeginInfo = wsiManager.get_render_targets()[imageIndex].get_render_pass_begin_info();
                vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor{ { }, renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
                    VkViewport viewport{ 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
                    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

                    // Bind the gvk::math::Camera uniform gvk::Buffer and the floor resources then
                    //  issue a draw call for the floor.  Then bind the floating cube resources...
                    //  we can leave the gvk::math::Camera uniform gvk::Buffer bound and update the
                    //  gvk::Pipeline and gvk::DescriptorSet at index 1 without distrubing the
                    //  gvk::DescriptorSet at index 0...then issue a draw call for the floating
                    //  cube...
                    auto pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                    const auto& imageExtent = wsiManager.get_render_targets()[imageIndex].get_image(0).get<VkImageCreateInfo>().extent;

                    // TODO : Draw sprites additively w/depth, then render grid

                    // Grid
                    shape_shooter::Context::instance().grid.record_draw_cmds(commandBuffer, shapeShooterContext.gameCamera, { (float)imageExtent.width, (float)imageExtent.height });

                    // CoordinateRenderer
                    const auto& gameCameraDescriptorSet = shapeShooterContext.gameCameraResources.second;
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, fontRendererPipelineLayout, 0, 1, &gameCameraDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
                    coordinateRenderer.record_draw_cmds(commandBuffer, shapeShooterContext.gameCamera, { (float)imageExtent.width, (float)imageExtent.height });

                    // Sprites
                    auto spriteCamera = shapeShooterContext.gameCamera;
                    // spriteCamera.projectionMode = gvk::math::Camera::ProjectionMode::Orthographic;
                    //spriteCamera.fieldOfView = viewport.width;
                    shape_shooter::Context::instance().spriteRenderer.record_draw_cmds(commandBuffer, spriteCamera);

                    // ScoreBoard
                    const auto& scoreBoardCameraDescriptorSet = shapeShooterContext.scoreBoardCameraResources.second;
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, fontRendererPipelineLayout, 0, 1, &scoreBoardCameraDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
                    shape_shooter::Context::instance().scoreBoard.record_draw_cmds(commandBuffer, shapeShooterContext.scoreBoardCamera);
                }

                // If the gvk::gui::Renderer is enabled, record cmds to render it
                if (showGui) {
                    guiRenderer.record_cmds(commandBuffer);
                }

                vkCmdEndRenderPass(commandBuffer);

                // Ensure the gvk::RenderTarget attachments are transitioned back to the
                //  VkImageLayout expected when the gvk::RenderPass is next executed...the
                //  VkImageMemoryBarrier objects provided by gvk::RenderTarget do not
                //  account for layout transitions that occur outside of the associated
                //  gvk::RenderPass, those must be handled by your application...
                auto attachmentCount = renderTarget.get_render_pass().get<VkRenderPassCreateInfo2>().attachmentCount;
                for (size_t attachment_i = 0; attachment_i < attachmentCount; ++attachment_i) {
                    auto imageMemoryBarrier = renderTarget.get_image_memory_barrier((uint32_t)attachment_i);
                    if (imageMemoryBarrier.oldLayout != imageMemoryBarrier.newLayout) {
                        vkCmdPipelineBarrier(
                            commandBuffer,
                            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                            0,
                            0, nullptr,
                            0, nullptr,
                            1, &imageMemoryBarrier
                        );
                    }
                }

                gvk_result(vkEndCommandBuffer(commandBuffer));

                const auto& queue = gvk::get_queue_family(device, 0).queues[0];
                auto submitInfo = wsiManager.get_submit_info(imageIndex);
                gvk_result(vkQueueSubmit(queue, 1, &submitInfo, vkFences[imageIndex]));

                auto presentInfo = wsiManager.get_present_info(&imageIndex);
                vkResult = vkQueuePresentKHR(gvk::get_queue_family(gvkContext.get_devices()[0], 0).queues[0], &presentInfo);
                gvk_result((vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR) ? VK_SUCCESS : vkResult);
            }
        }
        gvk_result(vkDeviceWaitIdle(gvkContext.get_devices()[0]));
    } gvk_result_scope_end;
    if (gvkResult) {
        std::cerr << gvk::to_string(gvkResult) << std::endl;
    }
    return (int)gvkResult;
}
