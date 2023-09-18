
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

#include "dynamic-static.graphics/sprite.hpp"
#include "dynamic-static/image.hpp"

#include "gvk-handles.hpp"

namespace dst {
namespace gfx {

VkResult load_image(const gvk::Context& gvkContext, const char* pFilePath, gvk::Buffer* pStagingBuffer, gvk::ImageView* pImageView)
{
    assert(gvkContext);
    assert(pFilePath);
    assert(pStagingBuffer);
    assert(pImageView);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        Image<> stagingImage;
        if (load_png(pFilePath, &stagingImage)) {
            // TODO : Documentation
            gvk_result(gvk::create_staging_buffer(gvkContext.get_devices()[0], stagingImage.size_bytes(), pStagingBuffer));
            uint8_t* pStagingData = nullptr;
            gvk_result(vmaMapMemory(gvkContext.get_devices()[0].get<VmaAllocator>(), pStagingBuffer->get<VmaAllocation>(), (void**)&pStagingData));
            memcpy(pStagingData, stagingImage.data(), stagingImage.size_bytes());
            vmaUnmapMemory(gvkContext.get_devices()[0].get<VmaAllocator>(), pStagingBuffer->get<VmaAllocation>());

            // TODO : Documentation
            auto imageCreateInfo = gvk::get_default<VkImageCreateInfo>();
            imageCreateInfo.extent.width = stagingImage.get_extent()[0];
            imageCreateInfo.extent.height = stagingImage.get_extent()[1];
            imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
            imageCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
            imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            auto allocationCreateInfo = gvk::get_default<VmaAllocationCreateInfo>();
            allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
            gvk::Image image;
            gvk_result(gvk::Image::create(gvkContext.get_devices()[0], &imageCreateInfo, &allocationCreateInfo, &image));

            // TODO : Documentation
            const auto& dispatchTable = gvkContext.get_devices()[0].get<gvk::DispatchTable>();
            gvk_result(gvk::execute_immediately(
                gvkContext.get_devices()[0],
                gvk::get_queue_family(gvkContext.get_devices()[0], 0).queues[0],
                gvkContext.get_command_buffers()[0],
                VK_NULL_HANDLE,
                [&](auto)
                {
                    auto imageMemoryBarrier = gvk::get_default<VkImageMemoryBarrier>();
                    imageMemoryBarrier.srcAccessMask = 0;
                    imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                    imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                    imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                    imageMemoryBarrier.image = image;
                    dispatchTable.gvkCmdPipelineBarrier(
                        gvkContext.get_command_buffers()[0],
                        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT,
                        0,
                        0, nullptr,
                        0, nullptr,
                        1, &imageMemoryBarrier
                    );

                    auto bufferImageCopy = gvk::get_default<VkBufferImageCopy>();
                    bufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    bufferImageCopy.imageExtent = imageCreateInfo.extent;
                    dispatchTable.gvkCmdCopyBufferToImage(gvkContext.get_command_buffers()[0], *pStagingBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &bufferImageCopy);

                    imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                    imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                    imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                    imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    dispatchTable.gvkCmdPipelineBarrier(
                        gvkContext.get_command_buffers()[0],
                        VK_PIPELINE_STAGE_TRANSFER_BIT,
                        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                        0,
                        0, nullptr,
                        0, nullptr,
                        1, &imageMemoryBarrier
                    );
                }
            ));

            // TODO : Documentation
            auto imageViewCreateInfo = gvk::get_default<VkImageViewCreateInfo>();
            imageViewCreateInfo.image = image;
            imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            imageViewCreateInfo.format = imageCreateInfo.format;
            gvk_result(gvk::ImageView::create(gvkContext.get_devices()[0], &imageViewCreateInfo, nullptr, pImageView));
        }
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<Sprite>::create(const gvk::Context& gvkContext, const CreateInfo& createInfo, Renderer<Sprite>* pRenderer)
{
    assert(createInfo.filePathCount);
    assert(createInfo.ppFilePaths);
    assert(pRenderer);
    pRenderer->reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk::Buffer stagingBuffer;
        for (uint32_t i = 0; i < createInfo.filePathCount; ++i) {
            gvk::ImageView imageView;
            gvk_result(load_image(gvkContext, createInfo.ppFilePaths[i], &stagingBuffer, &imageView));
            pRenderer->mImages[createInfo.ppFilePaths[i]] = imageView;
        }
    } gvk_result_scope_end;
    return gvkResult;
}

Renderer<Sprite>::~Renderer()
{
    reset();
}

void Renderer<Sprite>::reset()
{
    mImages.clear();
}

} // namespace gfx
} // namespace dst
