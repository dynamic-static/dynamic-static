
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

#include "dynamic-static.sample-utilities.hpp"

#include "dynamic-static/text.hpp"
#include "dynamic-static.graphics/text.hpp"

#include "stb/stb_image.h"

#include <map>

void load_image(const gvk::Context& context, const std::string& filePath, gvk::ImageView* pImageView)
{
    assert(pImageView);
    pImageView->reset();

    int width = 0;
    int height = 0;
    int channels = 0;
    stbi_uc* pData = stbi_load(filePath.c_str(), &width, &height, &channels, 4);
    assert(pData);

    auto imageCreateInfo = gvk::get_default<VkImageCreateInfo>();
    imageCreateInfo.extent.width = width;
    imageCreateInfo.extent.height = height;
    imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imageCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    VmaAllocationCreateInfo allocationCreateInfo { };
    allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    gvk::Image image;
    auto vkResult = gvk::Image::create(context.get_devices()[0], &imageCreateInfo, &allocationCreateInfo, &image);
    assert(vkResult == VK_SUCCESS);

    VmaAllocationInfo allocationInfo { };
    auto vmaAllocator = context.get_devices()[0].get<VmaAllocator>();
    vmaGetAllocationInfo(vmaAllocator, image.get<VmaAllocation>(), &allocationInfo);

    // Create the staging gvk::Buffer.
    //  Because we're using VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
    //  we should only perform sequential writes like memcpy().  See VMA's
    //  documentation for more info...
    //  https://gpuopen-librariesandsdks.github.io/VulkanMemoryAllocator/html/choosing_memory_type.html
    auto bufferCreateInfo = gvk::get_default<VkBufferCreateInfo>();
    bufferCreateInfo.size = allocationInfo.size;
    bufferCreateInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    gvk::Buffer stagingBuffer;
    vkResult = gvk::Buffer::create(context.get_devices()[0], &bufferCreateInfo, &allocationCreateInfo, &stagingBuffer);
    assert(vkResult == VK_SUCCESS);

    // Copy the image data that we got from stb into the gvk::Buffer...
    uint8_t* pStagingData = nullptr;
    vkResult = vmaMapMemory(vmaAllocator, stagingBuffer.get<VmaAllocation>(), (void**)&pStagingData);
    assert(vkResult == VK_SUCCESS);
    memcpy(pStagingData, pData, width * height * 4 * sizeof(uint8_t));
    vmaUnmapMemory(vmaAllocator, stagingBuffer.get<VmaAllocation>());

    stbi_image_free(pData);

    vkResult = gvk::execute_immediately(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        [&](auto)
        {
            // We start by recording a VkImageMemoryBarrier that will transition the
            //  gvk::Image to VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL and make it available at
            //  VK_PIPELINE_STAGE_TRANSFER_BIT...
            auto imageMemoryBarrier = gvk::get_default<VkImageMemoryBarrier>();
            imageMemoryBarrier.srcAccessMask = 0;
            imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            imageMemoryBarrier.image = image;
            vkCmdPipelineBarrier(
                context.get_command_buffers()[0],
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                0,
                0, nullptr,
                0, nullptr,
                1, &imageMemoryBarrier
            );

            // Copy from the staging resource to the gvk::Image...
            auto bufferImageCopy = gvk::get_default<VkBufferImageCopy>();
            bufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            bufferImageCopy.imageSubresource.layerCount = 1;
            bufferImageCopy.imageExtent = imageCreateInfo.extent;
            vkCmdCopyBufferToImage(context.get_command_buffers()[0], stagingBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &bufferImageCopy);

            // Then transition the gvk::Image to VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            //  after VK_PIPELINE_STAGE_TRANSFER_BIT and make it available at
            //  VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT since it will be sampled from during
            //  fragment shading...
            imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            vkCmdPipelineBarrier(
                context.get_command_buffers()[0],
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                0,
                0, nullptr,
                0, nullptr,
                1, &imageMemoryBarrier
            );
        }
    );
    assert(vkResult == VK_SUCCESS);

    // Create gvk::ImageView...
    auto imageViewCreateInfo = gvk::get_default<VkImageViewCreateInfo>();
    imageViewCreateInfo.image = image;
    imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    imageViewCreateInfo.format = imageCreateInfo.format;
    vkResult = gvk::ImageView::create(context.get_devices()[0], &imageViewCreateInfo, nullptr, pImageView);
    assert(vkResult == VK_SUCCESS);
}

std::map<std::string, gvk::ImageView> load_images(const gvk::Context& context)
{
    std::map<std::string, gvk::ImageView> images {
        { SHAPE_SHOOTER_CONTENT "/Art/Black Hole.png", VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Bullet.png",     VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Glow.png",       VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Laser.png",      VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Player.png",     VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Pointer.png",    VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Seeker.png",     VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Wanderer.png",   VK_NULL_HANDLE },
    };
    for (auto& image : images) {
        load_image(context, image.first, &image.second);
    }
    return images;
}

namespace shape_shooter {

class GameInfo final
{
public:
};

using GameState = dst::State<const GameInfo&>;

namespace game_state {

class Attract;
class Spawning;
class Playing;
class Celebration;
class GameOver;
class Resetting;

class Attract final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Spawning final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Playing final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Celebration final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class GameOver final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Resetting final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

} // namespace game_state
} // namespace shape_shooter

int main(int, const char* [])
{
    // Attract
    // Spawning
    // Playing
    // Celebration
    // GameOver
    // Resetting

    shape_shooter::GameState::Machine gameStateMachine;
    gameStateMachine.add_state<shape_shooter::game_state::Attract>();
    gameStateMachine.add_state<shape_shooter::game_state::Spawning>();
    gameStateMachine.add_state<shape_shooter::game_state::Playing>();
    gameStateMachine.add_state<shape_shooter::game_state::Celebration>();
    gameStateMachine.add_state<shape_shooter::game_state::GameOver>();
    gameStateMachine.add_state<shape_shooter::game_state::Resetting>();
    gameStateMachine.set_state<shape_shooter::game_state::Attract>();
    shape_shooter::GameInfo gameInfo { };
    gameStateMachine.update(gameInfo);

#if 0
    dst::Image<> image;
    auto& texel = image[{ 16, 32 }];
    (void)texel;
#endif

    // Create a gvk::Context.  This will initialize a VkInstance and VkDevice.
    gvk::Context gvkContext;
    dst_vk_result(dst_sample_create_gvk_context("dynamic-static - Shape Shooter", &gvkContext));
    auto gvkDevice = gvkContext.get_devices()[0];
    auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

    auto images = load_images(gvkContext);
    (void)images;

#if 0
    FMOD::System* pFmodSystem = nullptr;
    auto fmodResult = FMOD::System_Create(&pFmodSystem);
    assert(fmodResult == FMOD_OK);

    int audioDriverCount = 0;
    fmodResult = pFmodSystem->getNumDrivers(&audioDriverCount);
    assert(fmodResult == FMOD_OK);
    assert(audioDriverCount);

    fmodResult = pFmodSystem->init(36, FMOD_INIT_NORMAL, NULL);
    assert(fmodResult == FMOD_OK);

    FMOD::Sound* pFmodSound = nullptr;
    fmodResult = pFmodSystem->createSound(SHAPE_SHOOTER_CONTENT "/Audio/Music.mp3", FMOD_DEFAULT, 0, &pFmodSound);
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSound->setMode(FMOD_LOOP_NORMAL);
    assert(fmodResult == FMOD_OK);
    fmodResult = pFmodSound->setLoopCount(-1);
    assert(fmodResult == FMOD_OK);
    fmodResult = pFmodSystem->playSound(pFmodSound, nullptr, false, nullptr);
    assert(fmodResult == FMOD_OK);

    bool loop = true;
    while (loop) {
        int b = 0;
        (void)b;
    }

    fmodResult = pFmodSound->release();
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSystem->close();
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSystem->release();
    assert(fmodResult == FMOD_OK);
#endif

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
    dst_vk_result(gvk::WsiManager::create(gvkDevice, &wsiManagerCreateInfo, nullptr, &wsiManager));

    ///////////////////////////////////////////////////////////////////////////////



    std::shared_ptr<dst::text::Font> spFont;
    dst::text::Font::create("C:\\Windows\\Fonts\\georgia.ttf", nullptr, 16, &spFont);
    dst::gfx::Renderer<dst::text::Font> fontRenderer;
    dst::gfx::Renderer<dst::text::Font>::create(*spFont, wsiManager.get_render_pass(), &fontRenderer);

    dst::text::Mesh textMesh;
    textMesh.set_font(spFont);
    textMesh.set_text("The quick brown fox");
    textMesh.create_renderer<dst::gfx::Renderer<dst::text::Mesh>>(
        [&](const auto& /*textMesh*/, auto& renderer)
        {
            return dst::gfx::Renderer<dst::text::Mesh>::create(textMesh, &renderer);
        }
    );

    ///////////////////////////////////////////////////////////////////////////////

    while (
        !(systemSurface.get_input().keyboard.down(gvk::system::Key::Escape)) &&
        !(systemSurface.get_status() & gvk::system::Surface::CloseRequested)) {

        // Call the static function gvk::system::Surface::update() to cause all
        //  gvk::system::Surface objects to process window/input events.  Get a
        //  reference to the Surface's Input object.
        gvk::system::Surface::update();
        const auto& input = systemSurface.get_input();
        (void)input;

        // Call wsiManager.update().  This will cause WsiManager to respond to system
        //  updates for the SurfaceKHR it's managing.  This call may cause resources to
        //  be created/destroyed.  If there's a valid SwapchainKHR, render and present.
        wsiManager.update();
        auto swapchain = wsiManager.get_swapchain();
        if (swapchain) {
            // Acquire the next Image to render to.  The index will be used to access the
            //  acquired Image as well as the CommandBuffer and Fence associated with that
            //  Image.  Note that this method may return VK_SUBOPTIMAL_KHR when the window
            //  is resized/minimized/maximized/etc...gvk::WsiManager will update resources
            //  when this occurs so there's no need to bail.
            uint32_t imageIndex = 0;
            auto vkResult = wsiManager.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &imageIndex);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);

            // Using the acquired Image index, wait on the associated Fence.  This ensures
            //  that the Image isn't currently in use via vkQueueSubmit().  After waiting
            //  on the Fence, immediately reset it so that it's ready to be used in the
            //  next call to vkQueueSubmit().
            const auto& vkFences = wsiManager.get_vk_fences();
            dst_vk_result(vkWaitForFences(gvkDevice, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX));
            dst_vk_result(vkResetFences(gvkDevice, 1, &vkFences[imageIndex]));

            // Begin CommandBuffer recording and begin a RenderPass.
            const auto& commandBuffer = wsiManager.get_command_buffers()[imageIndex];
            dst_vk_result(vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>()));
            auto renderPassBeginInfo = wsiManager.get_render_targets()[imageIndex].get_render_pass_begin_info();
            vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            // Set the scissor and viewport to match the renderPassBeginInfo.renderArea
            VkRect2D scissor { .extent = renderPassBeginInfo.renderArea.extent };
            vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
            VkViewport viewport { .width = (float)scissor.extent.width, .height = (float)scissor.extent.height, .minDepth = 0, .maxDepth = 1 };
            vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

            {
                // TODO :
            }

            // End the RenderPass and CommandBuffer.
            vkCmdEndRenderPass(commandBuffer);
            dst_vk_result(vkEndCommandBuffer(commandBuffer));

            // Submit the CommandBuffer for execution on the GPU.
            auto submitInfo = wsiManager.get_submit_info(imageIndex);
            dst_vk_result(vkQueueSubmit(gvkQueue, 1, &submitInfo, vkFences[imageIndex]));

            // Present the SwapchainKHR Image that was drawn into.
            auto presentInfo = wsiManager.get_present_info(&imageIndex);
            vkResult = vkQueuePresentKHR(gvkQueue, &presentInfo);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);
        }
    }

    // Wait for the GPU to be idle before allowing graphics destructors to run.
    dst_vk_result(vkDeviceWaitIdle(gvkContext.get_devices()[0]));

    return 0;
}
