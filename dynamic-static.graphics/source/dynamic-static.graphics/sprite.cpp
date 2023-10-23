
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

#if 0

#include "dynamic-static/image.hpp"

#include "gvk-handles.hpp"

#include <algorithm>
#include <iostream>

#if defined(_WIN32) || defined(_WIN64)
#include <malloc.h>
#else
#include <errno.h>
#include <stdlib.h>
#endif // defined(_WIN32) || defined(_WIN64)

static VkResult dst_sample_allocate_descriptor_sets_HACK(const gvk::Pipeline& pipeline, uint32_t descriptorCount, std::vector<gvk::DescriptorSet>& descriptorSets)
{
    assert(pipeline);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        // Use the provided gvk::Pipeline's gvk::PipelineLayout to determine what types
        //  and how many descriptors we'll need...the samples generally allocate a very
        //  limited number of descriptors so this works fine...in real world scenario
        //  you'd likely employ a much more robust strategy for managing descriptors.
        //  Startegies for managing descriptors are highly application dependent...
        std::vector<VkDescriptorPoolSize> descriptorPoolSizes;
        std::vector<VkDescriptorSetLayout> vkDescriptorSetLayouts;
        for (const auto& descriptorSetLayout : pipeline.get<gvk::PipelineLayout>().get<gvk::DescriptorSetLayouts>()) {
            vkDescriptorSetLayouts.push_back(descriptorSetLayout);
            auto descriptorSetLayoutCreateInfo = descriptorSetLayout.get<VkDescriptorSetLayoutCreateInfo>();
            for (uint32_t i = 0; i < descriptorSetLayoutCreateInfo.bindingCount; ++i) {
                const auto& descriptorSetLayoutBinding = descriptorSetLayoutCreateInfo.pBindings[i];
                descriptorPoolSizes.push_back({
                    /* .type            = */ descriptorSetLayoutBinding.descriptorType,
                    /* .descriptorCount = */ descriptorSetLayoutBinding.descriptorCount
                });
            }
        }

        descriptorSets.clear();
        assert(vkDescriptorSetLayouts.empty() == descriptorPoolSizes.empty());
        if (!vkDescriptorSetLayouts.empty() && !descriptorPoolSizes.empty()) {
            // Create a gvk::DescriptorPool...
            auto descriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
            descriptorPoolCreateInfo.maxSets = (uint32_t)vkDescriptorSetLayouts.size();
            descriptorPoolCreateInfo.poolSizeCount = (uint32_t)descriptorPoolSizes.size();
            descriptorPoolCreateInfo.pPoolSizes = descriptorPoolSizes.data();
            gvk::DescriptorPool descriptorPool;
            gvk_result(gvk::DescriptorPool::create(pipeline.get<gvk::Device>(), &descriptorPoolCreateInfo, nullptr, &descriptorPool));

            // TODO : Extension
            auto descriptorSetVariableDescriptorCountAllocateInfo = gvk::get_default<VkDescriptorSetVariableDescriptorCountAllocateInfo>();
            descriptorSetVariableDescriptorCountAllocateInfo.descriptorSetCount = 1;
            descriptorSetVariableDescriptorCountAllocateInfo.pDescriptorCounts = &descriptorCount;

            // And allocate gvk::DescriptorSets...
            // NOTE : The allocated gvk::DescriptorSets will hold references to the
            //  gvk::DescriptorPool so there's no need for user code to maintain an
            //  explicit reference.  A gvk::DescriptorSet's gvk::DescriptorPool can be
            //  retrieved using descriptorSet.get<gvk::DescriptorPool>().
            // NOTE : vkResetDescriptorPool() must not be used with gvk::DescriptorSets.
            // NOTE : A gvk::DescriptorPool may be used to allocate VkDescriptorSets and
            //  use vkResetDescriptorPool() as normal.
            auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
            descriptorSetAllocateInfo.pNext = &descriptorSetVariableDescriptorCountAllocateInfo;
            descriptorSetAllocateInfo.descriptorPool = descriptorPool;
            descriptorSetAllocateInfo.descriptorSetCount = (uint32_t)vkDescriptorSetLayouts.size();
            descriptorSetAllocateInfo.pSetLayouts = vkDescriptorSetLayouts.data();
            descriptorSets.resize(descriptorSetAllocateInfo.descriptorSetCount);
            gvk_result(gvk::DescriptorSet::allocate(pipeline.get<gvk::Device>(), &descriptorSetAllocateInfo, descriptorSets.data()));
        }
    } gvk_result_scope_end;
    return gvkResult;
}

namespace dst {
namespace gfx {

/**
Allocates memory on a given alignment boundary
@param [in] size The size of the requested allocation
@param [in] The alignment value, which must be a multiple of sizeof(void*) and an integer power of 2
@return A pointer to the memory block that was allocated or nullptr if the operation failed
*/
inline void* aligned_malloc(size_t size, size_t alignment)
{
    void* ptr = nullptr;
    #if defined(_WIN32) || defined(_WIN64)
    ptr = _aligned_malloc(size, alignment);
    #else
    // NOTE : https://man7.org/linux/man-pages/man3/posix_memalign.3.html
    ptr = nullptr;
    switch (posix_memalign(&ptr, alignment, size)) {
    case EINVAL: {
        assert(false && "The alignment argument was not a power of tow, or was not a multiple of sizeof(void*)");
    } break;
    case ENOMEM: {
        assert(false && "There was insufficient memory to fulfill the allocation");
    } break;
    default: {
    } break;
    }
    #endif // defined(_WIN32) || defined(_WIN64)
    return ptr;
}

/**
Frees a block of memory that was allocated with dst::gfx::aligned_malloc()
@param [in] ptr A pointer to the memory block to free
*/
inline void aligned_free(void* ptr)
{
    #if defined(_WIN32) || defined(_WIN64)
    _aligned_free(ptr);
    #else
    free(ptr);
    #endif // defined(_WIN32) || defined(_WIN64)
}

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

VkResult Renderer<Sprite>::create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, const CreateInfo& createInfo, Renderer<Sprite>* pRenderer)
{
    assert(createInfo.filePathCount);
    assert(createInfo.ppFilePaths);
    assert(pRenderer);
    pRenderer->reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(pRenderer->create_image_views(gvkContext, createInfo));
        gvk_result(pRenderer->create_pipeline(gvkContext, renderPass));
        gvk_result(pRenderer->allocate_descriptor_set(gvkContext));
    } gvk_result_scope_end;
    return gvkResult;
}

Renderer<Sprite>::~Renderer()
{
    reset();
}

void Renderer<Sprite>::reset()
{
    mPipeline.reset();
    mStorageBuffer.reset();
    mImages.clear();
    mDescriptorSet.reset();
    mSprites.clear();
}

const std::vector<std::pair<std::string, gvk::ImageView>>& Renderer<Sprite>::get_images() const
{
    return mImages;
}

void Renderer<Sprite>::begin_sprite_batch()
{
    mSprites.clear();
}

void Renderer<Sprite>::submit(const Sprite& sprite)
{
    // TODO : Cache this...
    const auto& imageCreateInfo = mImages[sprite.imageIndex].second.get<gvk::Image>().get<VkImageCreateInfo>();
    GlSprite glSprite { };
    glSprite.extent = { imageCreateInfo.extent.width, imageCreateInfo.extent.height, 0, 1 };
    glSprite.uvMin = { 0, 0, 0, (float)sprite.imageIndex };
    glSprite.uvMax = { 1, 1, 0, 0 };
    glSprite.color = sprite.color;
    glSprite.model = sprite.transform.world_from_local();
    mSprites.push_back(glSprite);
}

void Renderer<Sprite>::end_sprite_batch()
{
    assert(mPipeline);
    if (!mSprites.empty()) {
        gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
            const auto& device = mPipeline.get<gvk::Device>();
            auto size = (VkDeviceSize)(mSprites.size() * sizeof(GlSprite));
            auto bufferCreateInfo = mStorageBuffer ? mStorageBuffer.get<VkBufferCreateInfo>() : gvk::get_default<VkBufferCreateInfo>();
            if (bufferCreateInfo.size < size) {
                // HACK :
                gvk_result(device.get<gvk::DispatchTable>().gvkDeviceWaitIdle(device));

                // Create VkBuffer
                bufferCreateInfo.size = size;
                bufferCreateInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                auto allocationCreateInfo = gvk::get_default<VmaAllocationCreateInfo>();
                allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
                allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
                gvk_result(gvk::Buffer::create(device, &bufferCreateInfo, &allocationCreateInfo, &mStorageBuffer));

#if 0
                // Update VkDescriptorSet
                auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
                descriptorBufferInfo.buffer = mStorageBuffer;
                auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
                writeDescriptorSet.dstSet = mDescriptorSet;
                writeDescriptorSet.descriptorCount = 1;
                writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
                device.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);
#endif

                std::array<VkWriteDescriptorSet, 2> writeDescriptorSets {
                    gvk::get_default<VkWriteDescriptorSet>(),
                    gvk::get_default<VkWriteDescriptorSet>(),
                };
                // Storage buffer
                auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
                descriptorBufferInfo.buffer = mStorageBuffer;
                writeDescriptorSets[0].dstSet = mDescriptorSet;
                writeDescriptorSets[0].dstBinding = 0;
                writeDescriptorSets[0].descriptorCount = 1;
                writeDescriptorSets[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                writeDescriptorSets[0].pBufferInfo = &descriptorBufferInfo;
                // Image array
                std::vector<VkDescriptorImageInfo> descriptorImageInfos;
                descriptorImageInfos.reserve(mImages.size());
                for (const auto& image : mImages) {
                    auto descriptorImageInfo = gvk::get_default<VkDescriptorImageInfo>();
                    descriptorImageInfo.sampler = mSampler;
                    descriptorImageInfo.imageView = image.second;
                    descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    descriptorImageInfos.push_back(descriptorImageInfo);
                }
                writeDescriptorSets[1].dstSet = mDescriptorSet;
                writeDescriptorSets[1].dstBinding = 1;
                writeDescriptorSets[1].descriptorCount = (uint32_t)mImages.size();
                writeDescriptorSets[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writeDescriptorSets[1].pImageInfo = descriptorImageInfos.data();
                device.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(device, (uint32_t)writeDescriptorSets.size(), writeDescriptorSets.data(), 0, nullptr);
            }

            // Update VkBuffer
            auto vmaAllocator = device.get<VmaAllocator>();
            auto vmaAllocation = mStorageBuffer.get<VmaAllocation>();
            uint8_t* pData = nullptr;
            gvk_result(vmaMapMemory(vmaAllocator, vmaAllocation, (void**)&pData));
            memcpy(pData, mSprites.data(), size);
            gvk_result(vmaFlushAllocation(vmaAllocator, vmaAllocation, 0, size));
            vmaUnmapMemory(vmaAllocator, vmaAllocation);
        } gvk_result_scope_end;
        assert(gvkResult == VK_SUCCESS);
    }
}

void Renderer<Sprite>::record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera) const
{
    if (!mSprites.empty()) {
        auto bindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        const auto& pipelineLayout = mPipeline.get<gvk::PipelineLayout>();
        const auto& dispatchTable = commandBuffer.get<gvk::Device>().get<gvk::DispatchTable>();
        dispatchTable.gvkCmdBindPipeline(commandBuffer, bindPoint, mPipeline);
        std::array<glm::mat4, 2> cameraMatrices { camera.view(), camera.projection() };
        dispatchTable.gvkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(cameraMatrices), cameraMatrices.data());
        dispatchTable.gvkCmdBindDescriptorSets(commandBuffer, bindPoint, pipelineLayout, 0, 1, &mDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
        dispatchTable.gvkCmdDraw(commandBuffer, 4, (uint32_t)mSprites.size(), 0, 0);
    }
}

static VkResult validate_shader_info(const gvk::spirv::ShaderInfo& shaderInfo)
{
    if (!shaderInfo.errors.empty()) {
        for (const auto& error : shaderInfo.errors) {
            std::cerr << error << std::endl;
        }
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    return VK_SUCCESS;
}

VkResult Renderer<Sprite>::create_pipeline(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass)
{
    assert(gvkContext);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        auto vertexShaderInfo = gvk::get_default<gvk::spirv::ShaderInfo>();
        vertexShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        vertexShaderInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderInfo.lineOffset = __LINE__;
        vertexShaderInfo.source = R"(
            #version 460

            struct Sprite
            {
                vec4 extent;
                vec4 uvMin;
                vec4 uvMax;
                vec4 color;
                mat4 model;
            };

            layout(push_constant) uniform Camera
            {
                mat4 view;
                mat4 projection;
            } camera;

            layout(std140, set = 0, binding = 0)
            readonly buffer SpriteBuffer
            {
                Sprite sprites[];
            } spriteBuffer;

            vec4 Vertices[4] = vec4[](
                vec4(-0.45,  0.5, 0, 1),
                vec4( 0.45,  0.5, 0, 1),
                vec4(-0.45, -0.5, 0, 1),
                vec4( 0.45, -0.5, 0, 1)
            );

            layout(location = 0) out float fsTexindex;
            layout(location = 1) out vec2 fsTexcoord;
            layout(location = 2) out vec4 fsColor;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };

            void main()
            {
                Sprite sprite = spriteBuffer.sprites[gl_InstanceIndex];
                vec4 position = Vertices[gl_VertexIndex] * sprite.extent;
                gl_Position = camera.projection * camera.view * sprite.model * position;
                vec2 texcoords[4] = vec2[](
                    vec2(sprite.uvMin.x, sprite.uvMax.y),
                    vec2(sprite.uvMax.x, sprite.uvMax.y),
                    vec2(sprite.uvMin.x, sprite.uvMin.y),
                    vec2(sprite.uvMax.x, sprite.uvMin.y)
                );
                fsTexindex = sprite.uvMin.w;
                fsTexcoord = texcoords[gl_VertexIndex];
                fsColor = sprite.color;
            }
        )";

        auto fragmentShaderInfo = gvk::get_default<gvk::spirv::ShaderInfo>();
        fragmentShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        fragmentShaderInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderInfo.lineOffset = __LINE__;
        fragmentShaderInfo.source = R"(
            #version 450

            #extension GL_EXT_nonuniform_qualifier : enable

            layout(set = 0, binding = 1) uniform sampler2D images[];

            layout(location = 0) in float fsTexindex;
            layout(location = 1) in vec2 fsTexcoord;
            layout(location = 2) in vec4 fsColor;

            layout(location = 0) out vec4 fragColor;

            void main()
            {
                fragColor = texture(images[nonuniformEXT(int(fsTexindex))], fsTexcoord) * fsColor;
            }
        )";

        gvk::spirv::Context spirvContext;
        gvk_result(gvk::spirv::Context::create(&gvk::get_default<gvk::spirv::Context::CreateInfo>(), &spirvContext));
        auto vsVkResult = spirvContext.compile(&vertexShaderInfo);
        auto fsVkResult = spirvContext.compile(&fragmentShaderInfo);
        vsVkResult = validate_shader_info(vertexShaderInfo);
        fsVkResult = validate_shader_info(fragmentShaderInfo);
        gvk_result(vsVkResult);
        gvk_result(fsVkResult);

        auto vertexShaderModuleCreateInfo = gvk::get_default<VkShaderModuleCreateInfo>();
        vertexShaderModuleCreateInfo.codeSize = vertexShaderInfo.spirv.size() * sizeof(uint32_t);
        vertexShaderModuleCreateInfo.pCode = !vertexShaderInfo.spirv.empty() ? vertexShaderInfo.spirv.data() : nullptr;
        gvk::ShaderModule vertexShaderModule;
        gvk_result(gvk::ShaderModule::create(gvkContext.get_devices()[0], &vertexShaderModuleCreateInfo, nullptr, &vertexShaderModule));
        auto vertexPipelineShaderStageCreateInfo = gvk::get_default<VkPipelineShaderStageCreateInfo>();
        vertexPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexPipelineShaderStageCreateInfo.module = vertexShaderModule;

        auto fragmentShaderModuleCreateInfo = gvk::get_default<VkShaderModuleCreateInfo>();
        fragmentShaderModuleCreateInfo.codeSize = fragmentShaderInfo.spirv.size() * sizeof(uint32_t);
        fragmentShaderModuleCreateInfo.pCode = !fragmentShaderInfo.spirv.empty() ? fragmentShaderInfo.spirv.data() : nullptr;
        gvk::ShaderModule fragmentShaderModule;
        gvk_result(gvk::ShaderModule::create(gvkContext.get_devices()[0], &fragmentShaderModuleCreateInfo, nullptr, &fragmentShaderModule));
        auto fragmentPipelineShaderStageCreateInfo = gvk::get_default<VkPipelineShaderStageCreateInfo>();
        fragmentPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentPipelineShaderStageCreateInfo.module = fragmentShaderModule;

        std::array<VkPipelineShaderStageCreateInfo, 2> pipelineShaderStageCreateInfos {
            vertexPipelineShaderStageCreateInfo,
            fragmentPipelineShaderStageCreateInfo,
        };

        auto pipelineInputAssemblyStateCreateInfo = gvk::get_default<VkPipelineInputAssemblyStateCreateInfo>();
        pipelineInputAssemblyStateCreateInfo.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;

        auto pipelineColorBlendAttachmentState = gvk::get_default<VkPipelineColorBlendAttachmentState>();
        pipelineColorBlendAttachmentState.blendEnable = VK_TRUE;
        auto pipelineColorBlendStateCreateInfo = gvk::get_default<VkPipelineColorBlendStateCreateInfo>();
        pipelineColorBlendStateCreateInfo.pAttachments = &pipelineColorBlendAttachmentState;

        auto pipelineMultisampleStateCreateInfo = gvk::get_default<VkPipelineMultisampleStateCreateInfo>();
        pipelineMultisampleStateCreateInfo.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        auto renderPassCreateInfo = renderPass.get<VkRenderPassCreateInfo>();
        if (renderPassCreateInfo.sType == gvk::get_stype<VkRenderPassCreateInfo>()) {
            for (uint32_t i = 0; i < renderPassCreateInfo.attachmentCount; ++i) {
                pipelineMultisampleStateCreateInfo.rasterizationSamples = std::max(pipelineMultisampleStateCreateInfo.rasterizationSamples, renderPassCreateInfo.pAttachments[i].samples);
            }
        } else {
            auto renderPassCreateInfo2 = renderPass.get<VkRenderPassCreateInfo2>();
            for (uint32_t i = 0; i < renderPassCreateInfo2.attachmentCount; ++i) {
                pipelineMultisampleStateCreateInfo.rasterizationSamples = std::max(pipelineMultisampleStateCreateInfo.rasterizationSamples, renderPassCreateInfo2.pAttachments[i].samples);
            }
        }

        auto pipelineDepthStencilStateCreateInfo = gvk::get_default<VkPipelineDepthStencilStateCreateInfo>();

        auto pipelineRasterizationStateCreateInfo = gvk::get_default<VkPipelineRasterizationStateCreateInfo>();
        pipelineRasterizationStateCreateInfo.cullMode = VK_CULL_MODE_NONE;

        gvk::spirv::BindingInfo spirvBindingInfo;
        spirvBindingInfo.add_shader(vertexShaderInfo);
        spirvBindingInfo.add_shader(fragmentShaderInfo);

        // TODO : It would be good to figure out how to get this info from SPIRV-Cross
        std::array<VkDescriptorBindingFlags, 2> descriptorBindingFlags { 0, VK_DESCRIPTOR_BINDING_VARIABLE_DESCRIPTOR_COUNT_BIT };
        auto descriptorSetLayoutBindingFlagsCreateInfo = gvk::get_default<VkDescriptorSetLayoutBindingFlagsCreateInfo>();
        descriptorSetLayoutBindingFlagsCreateInfo.bindingCount = (uint32_t)descriptorBindingFlags.size();
        descriptorSetLayoutBindingFlagsCreateInfo.pBindingFlags = descriptorBindingFlags.data();
        auto descriptorSetLayoutCreateInfo = gvk::get_default<VkDescriptorSetLayoutCreateInfo>();
        descriptorSetLayoutCreateInfo.pNext = &descriptorSetLayoutBindingFlagsCreateInfo;
        spirvBindingInfo.descriptorSetLayoutCreateInfos[0] = descriptorSetLayoutCreateInfo;
        spirvBindingInfo.descriptorSetLayoutBindings[0][1].descriptorCount = (uint32_t)mImages.size();

        gvk::PipelineLayout pipelineLayout;
        gvk_result(gvk::spirv::create_pipeline_layout(gvkContext.get_devices()[0], spirvBindingInfo, nullptr, &pipelineLayout));

        auto graphicsPipelineCreateInfo = gvk::get_default<VkGraphicsPipelineCreateInfo>();
        graphicsPipelineCreateInfo.stageCount = (uint32_t)pipelineShaderStageCreateInfos.size();
        graphicsPipelineCreateInfo.pStages = pipelineShaderStageCreateInfos.data();
        graphicsPipelineCreateInfo.pInputAssemblyState = &pipelineInputAssemblyStateCreateInfo;
        graphicsPipelineCreateInfo.pColorBlendState = &pipelineColorBlendStateCreateInfo;
        graphicsPipelineCreateInfo.pMultisampleState = &pipelineMultisampleStateCreateInfo;
        graphicsPipelineCreateInfo.pDepthStencilState = &pipelineDepthStencilStateCreateInfo;
        graphicsPipelineCreateInfo.pRasterizationState = &pipelineRasterizationStateCreateInfo;
        graphicsPipelineCreateInfo.layout = pipelineLayout;
        graphicsPipelineCreateInfo.renderPass = renderPass;
        gvk_result(gvk::Pipeline::create(gvkContext.get_devices()[0], VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, nullptr, &mPipeline));
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<Sprite>::create_image_views(const gvk::Context& gvkContext, const CreateInfo& createInfo)
{
    assert(gvkContext);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk::Buffer stagingBuffer;
        mImages.resize(createInfo.filePathCount);
        for (uint32_t i = 0; i < createInfo.filePathCount; ++i) {
            gvk::ImageView imageView;
            gvk_result(load_image(gvkContext, createInfo.ppFilePaths[i], &stagingBuffer, &imageView));
            mImages[i].first = createInfo.ppFilePaths[i];
            mImages[i].second = imageView;
        }
        gvk_result(gvk::Sampler::create(gvkContext.get_devices()[0], &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &mSampler));
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<Sprite>::allocate_descriptor_set(const gvk::Context& gvkContext)
{
    (void)gvkContext;
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        std::vector<gvk::DescriptorSet> descriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets_HACK(mPipeline, (uint32_t)mImages.size(), descriptorSets));
        assert(descriptorSets.size() == 1);
        mDescriptorSet = descriptorSets[0];
    } gvk_result_scope_end;
    return gvkResult;
}

} // namespace gfx
} // namespace dst

#endif
