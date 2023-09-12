
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

#include "dynamic-static.graphics/text.hpp"

#include <iostream>

struct CameraUniforms
{
    glm::mat4 view { };
    glm::mat4 projection { };
};

struct TextMeshUniforms
{
    glm::mat4 world { };
};

template <typename FunctionType>
inline VkResult dvk_result_scope(FunctionType function)
{
    return function();
}

#ifndef dvk_error
#define dvk_error(DVK_RESULT, DVK_CALL) \
assert(DVK_RESULT == VK_SUCCESS && #DVK_CALL);
#endif

#define dvk_result_scope_begin(DVK_RESULT)  \
dvk_result_scope([&]()                      \
{                                           \
    auto dvkResult = DVK_RESULT;            \

#define dvk_result(DVK_CALL)            \
{                                       \
    dvkResult = (DVK_CALL);             \
    if (dvkResult != VK_SUCCESS) {      \
        dvk_error(dvkResult, DVK_CALL); \
        return dvkResult;               \
    }                                   \
}

#define dvk_result_scope_end \
        return dvkResult;    \
    }                        \
);

inline VkResult dst_sample_allocate_descriptor_sets_HACK(const gvk::Pipeline& pipeline, std::vector<gvk::DescriptorSet>& descriptorSets)
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

            // And allocate gvk::DescriptorSets...
            // NOTE : The allocated gvk::DescriptorSets will hold references to the
            //  gvk::DescriptorPool so there's no need for user code to maintain an
            //  explicit reference.  A gvk::DescriptorSet's gvk::DescriptorPool can be
            //  retrieved using descriptorSet.get<gvk::DescriptorPool>().
            // NOTE : vkResetDescriptorPool() must not be used with gvk::DescriptorSets.
            // NOTE : A gvk::DescriptorPool may be used to allocate VkDescriptorSets and
            //  use vkResetDescriptorPool() as normal.
            auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
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

VkResult Renderer<dst::text::Font>::create(const gvk::Context& context, const gvk::RenderPass& renderPass, const dst::text::Font& font, Renderer<dst::text::Font>* pRenderer)
{
    assert(renderPass);
    assert(pRenderer);
    return dvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        dvk_result(pRenderer->create_pipline(renderPass, font));
        dvk_result(gvk::Sampler::create(context.get_devices()[0], &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &pRenderer->mSampler));
        dvk_result(pRenderer->create_image_views(context, font));
        dvk_result(pRenderer->allocate_descriptor_set());
        pRenderer->update_descriptor_set();
    } dvk_result_scope_end;
}

const gvk::Pipeline& Renderer<dst::text::Font>::get_pipeline() const
{
    return mPipeline;
}

const gvk::DescriptorSet& Renderer<dst::text::Font>::get_descriptor_set() const
{
    return mDescriptorSet;
}

void Renderer<dst::text::Font>::record_bind_cmds(const gvk::CommandBuffer& commandBuffer)
{
    (void)commandBuffer;
    assert(false);
#if 0
    assert(commandBuffer);
    const auto& device = commandBuffer.get<gvk::Device>();
    const auto& dispatchTable = device.get<gvk::DispatchTable>();
    auto bindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    dispatchTable.gvkCmdBindPipeline(commandBuffer, bindPoint, mPipeline);
    dispatchTable.gvkCmdBindDescriptorSets(commandBuffer, bindPoint, mPipeline.get<gvk::PipelineLayout>(), 0, 1, &mDescriptorSet.get<const VkDescriptorSet&>(), 0, nullptr);
#endif
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

VkResult Renderer<dst::text::Font>::create_pipline(const gvk::RenderPass& renderPass, const dst::text::Font& font)
{
    (void)font;
    assert(renderPass);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        const auto& device = renderPass.get<gvk::Device>();

        auto vertexShaderInfo = gvk::get_default<gvk::spirv::ShaderInfo>();
        vertexShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        vertexShaderInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderInfo.lineOffset = __LINE__;
        vertexShaderInfo.source = R"(
            #version 450

            layout(set = 0, binding = 0)
            uniform CameraUniforms
            {
                mat4 view;
                mat4 projection;
            } camera;

            layout(set = 2, binding = 0)
            uniform ObjectUniforms
            {
                mat4 world;
            } object;

            layout(location = 0) in vec3 vsPosition;
            layout(location = 1) in vec2 vsTexcoord;
            layout(location = 2) in vec4 vsColor;
            layout(location = 0) out vec2 fsTexcoord;
            layout(location = 1) out vec4 fsColor;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };

            void main()
            {
                gl_Position = camera.projection * camera.view * object.world * vec4(vsPosition, 1);
                fsTexcoord = vsTexcoord;
                fsColor = vsColor;
            }
        )";

        auto fragmentShaderInfo = gvk::get_default<gvk::spirv::ShaderInfo>();
        fragmentShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        fragmentShaderInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderInfo.lineOffset = __LINE__;
        fragmentShaderInfo.source = R"(
            #version 450

            layout(set = 1, binding = 0) uniform sampler2D image;

            layout(set = 2, binding = 0)
            uniform ObjectUniforms
            {
                mat4 world;
            } object;

            layout(location = 0) in vec2 fsTexcoord;
            layout(location = 1) in vec4 fsColor;
            layout(location = 0) out vec4 fragColor;

            void main()
            {
                fragColor = texture(image, fsTexcoord) * fsColor;
                // fragColor = vec4(1, 1, 1, 1);
            }
        )";

        gvk::spirv::Context spirvContext;
        gvk_result(gvk::spirv::Context::create(&gvk::get_default<gvk::spirv::Context::CreateInfo>(), &spirvContext));
        gvk_result(spirvContext.compile(&vertexShaderInfo));
        gvk_result(spirvContext.compile(&fragmentShaderInfo));
        auto vsVkResult = validate_shader_info(vertexShaderInfo);
        auto fsVkResult = validate_shader_info(fragmentShaderInfo);
        gvk_result(vsVkResult);
        gvk_result(fsVkResult);

        auto vertexShaderModuleCreateInfo = gvk::get_default<VkShaderModuleCreateInfo>();
        vertexShaderModuleCreateInfo.codeSize = vertexShaderInfo.spirv.size() * sizeof(uint32_t);
        vertexShaderModuleCreateInfo.pCode = !vertexShaderInfo.spirv.empty() ? vertexShaderInfo.spirv.data() : nullptr;
        gvk::ShaderModule vertexShaderModule;
        gvk_result(gvk::ShaderModule::create(device, &vertexShaderModuleCreateInfo, nullptr, &vertexShaderModule));
        auto vertexPipelineShaderStageCreateInfo = gvk::get_default<VkPipelineShaderStageCreateInfo>();
        vertexPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexPipelineShaderStageCreateInfo.module = vertexShaderModule;

        auto fragmentShaderModuleCreateInfo = gvk::get_default<VkShaderModuleCreateInfo>();
        fragmentShaderModuleCreateInfo.codeSize = fragmentShaderInfo.spirv.size() * sizeof(uint32_t);
        fragmentShaderModuleCreateInfo.pCode = !fragmentShaderInfo.spirv.empty() ? fragmentShaderInfo.spirv.data() : nullptr;
        gvk::ShaderModule fragmentShaderModule;
        gvk_result(gvk::ShaderModule::create(device, &fragmentShaderModuleCreateInfo, nullptr, &fragmentShaderModule));
        auto fragmentPipelineShaderStageCreateInfo = gvk::get_default<VkPipelineShaderStageCreateInfo>();
        fragmentPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentPipelineShaderStageCreateInfo.module = fragmentShaderModule;

        std::array<VkPipelineShaderStageCreateInfo, 2> pipelineShaderStageCreateInfos {
            vertexPipelineShaderStageCreateInfo,
            fragmentPipelineShaderStageCreateInfo,
        };

        VkVertexInputBindingDescription vertexInputBindingDescription { 0, sizeof(dst::text::Mesh::Vertex), VK_VERTEX_INPUT_RATE_VERTEX };
        auto vertexInputAttributeDescriptions = gvk::get_vertex_description<dst::text::Mesh::Vertex>(0);
        auto pipelineVertexInputStateCreateInfo = gvk::get_default<VkPipelineVertexInputStateCreateInfo>();
        pipelineVertexInputStateCreateInfo.vertexBindingDescriptionCount = 1;
        pipelineVertexInputStateCreateInfo.pVertexBindingDescriptions = &vertexInputBindingDescription;
        pipelineVertexInputStateCreateInfo.vertexAttributeDescriptionCount = (uint32_t)vertexInputAttributeDescriptions.size();
        pipelineVertexInputStateCreateInfo.pVertexAttributeDescriptions = vertexInputAttributeDescriptions.data();

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
        gvk::PipelineLayout pipelineLayout;
        gvk_result(gvk::spirv::create_pipeline_layout(device, spirvBindingInfo, nullptr, &pipelineLayout));

        auto graphicsPipelineCreateInfo = gvk::get_default<VkGraphicsPipelineCreateInfo>();
        graphicsPipelineCreateInfo.stageCount = (uint32_t)pipelineShaderStageCreateInfos.size();
        graphicsPipelineCreateInfo.pStages = pipelineShaderStageCreateInfos.data();
        graphicsPipelineCreateInfo.pVertexInputState = &pipelineVertexInputStateCreateInfo;
        graphicsPipelineCreateInfo.pColorBlendState = &pipelineColorBlendStateCreateInfo;
        graphicsPipelineCreateInfo.pMultisampleState = &pipelineMultisampleStateCreateInfo;
        graphicsPipelineCreateInfo.pDepthStencilState = &pipelineDepthStencilStateCreateInfo;
        graphicsPipelineCreateInfo.pRasterizationState = &pipelineRasterizationStateCreateInfo;
        graphicsPipelineCreateInfo.layout = pipelineLayout;
        graphicsPipelineCreateInfo.renderPass = renderPass;
        gvk_result(gvk::Pipeline::create(device, VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, nullptr, &mPipeline));
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<dst::text::Font>::create_image_views(const gvk::Context& context, const dst::text::Font& font)
{
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        const auto& device = context.get_devices()[0];
        gvk_result(gvk::Sampler::create(device, &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &mSampler));
        gvk::Buffer stagingBuffer;
        mImageViews.reserve(font.get_atlas().pages.size());
        for (const auto& page : font.get_atlas().pages) {

            // TODO : Documentation
            auto imageCreateInfo = gvk::get_default<VkImageCreateInfo>();
            imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
            imageCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
            imageCreateInfo.extent.width = page.get_extent()[0];
            imageCreateInfo.extent.height = page.get_extent()[1];
            imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            auto allocationCreateInfo = gvk::get_default<VmaAllocationCreateInfo>();
            allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
            gvk::Image image;
            gvk_result(gvk::Image::create(device, &imageCreateInfo, &allocationCreateInfo, &image));

            // TODO : Documentation
            VmaAllocationInfo allocationInfo { };
            auto vmaAllocator = device.get<VmaAllocator>();
            vmaGetAllocationInfo(vmaAllocator, image.get<VmaAllocation>(), &allocationInfo);
            gvk_result(gvk::create_staging_buffer(device, allocationInfo.size, &stagingBuffer));

            // TODO : Documentation
            uint8_t* pStagingData = nullptr;
            gvk_result(vmaMapMemory(vmaAllocator, stagingBuffer.get<VmaAllocation>(), (void**)&pStagingData));
            memcpy(pStagingData, page.data(), page.size_bytes());
            vmaUnmapMemory(vmaAllocator, stagingBuffer.get<VmaAllocation>());

            // TODO : Documentation
            gvk_result(gvk::execute_immediately(
                device,
                gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
                context.get_command_buffers()[0],
                VK_NULL_HANDLE,
                [&](const auto& commandBuffer)
                {
                    // TODO : Documentation
                    auto imageMemoryBarrier = gvk::get_default<VkImageMemoryBarrier>();
                    imageMemoryBarrier.srcAccessMask = 0;
                    imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                    imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                    imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                    imageMemoryBarrier.image = image;
                    vkCmdPipelineBarrier(
                        commandBuffer,
                        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT,
                        0,
                        0, nullptr,
                        0, nullptr,
                        1, &imageMemoryBarrier
                    );

                    // TODO : Documentation
                    auto bufferImageCopy = gvk::get_default<VkBufferImageCopy>();
                    bufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    bufferImageCopy.imageSubresource.layerCount = 1;
                    bufferImageCopy.imageExtent = imageCreateInfo.extent;
                    vkCmdCopyBufferToImage(commandBuffer, stagingBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &bufferImageCopy);

                    // TODO : Documentation
                    imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                    imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                    imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                    imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    vkCmdPipelineBarrier(
                        commandBuffer,
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
            gvk::ImageView imageView;
            gvk_result(gvk::ImageView::create(device, &imageViewCreateInfo, nullptr, &imageView));
            mImageViews.push_back(imageView);
        }
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<dst::text::Font>::allocate_descriptor_set()
{
    assert(mPipeline);
    return dvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        std::vector<gvk::DescriptorSet> descriptorSets;
        dvk_result(dst_sample_allocate_descriptor_sets_HACK(mPipeline, descriptorSets));
        assert(descriptorSets.size() == 3);
        mDescriptorSet = descriptorSets[1];
        assert(mDescriptorSet);
    } dvk_result_scope_end;
}

void Renderer<dst::text::Font>::update_descriptor_set()
{
    auto descriptorImageInfo = gvk::get_default<VkDescriptorImageInfo>();
    descriptorImageInfo.sampler = mSampler;
    descriptorImageInfo.imageView = mImageViews[0];
    descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
    writeDescriptorSet.dstSet = mDescriptorSet;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet.pImageInfo = &descriptorImageInfo;
    const auto& device = mPipeline.get<gvk::Device>();
    device.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);
}

VkResult Renderer<dst::text::Mesh>::create(const gvk::Device& device, const dst::text::Mesh& textMesh, const Renderer<dst::text::Font>& fontRenderer, Renderer<dst::text::Mesh>* pRenderer)
{
    (void)textMesh;
    pRenderer->mDevice = device;
    return dvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        dvk_result(pRenderer->create_uniform_buffer());
        dvk_result(pRenderer->allocate_descriptor_set(fontRenderer));
        pRenderer->update_descriptor_set();
    } dvk_result_scope_end;
}

void Renderer<dst::text::Mesh>::update(float deltaTime, const dst::text::Mesh& textMesh)
{
    (void)deltaTime;
    assert(mDevice);
    auto vkResult = dvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        const auto& vertices = textMesh.get_vertices();
        const auto& indices = textMesh.get_indices();
        mIndexCount = (uint32_t)indices.size();
        auto vertexDataSize = vertices.size() * sizeof(vertices[0]);
        mIndexDataOffset = vertexDataSize;
        auto indexDataSize = indices.size() * sizeof(indices[0]);
        auto vertexIndexDataSize = vertexDataSize + indexDataSize;
        if (!mVertexIndexBuffer || mVertexIndexBuffer.get<VkBufferCreateInfo>().size < vertexIndexDataSize) {
            auto bufferCreateInfo = gvk::get_default<VkBufferCreateInfo>();
            bufferCreateInfo.size = vertexIndexDataSize;
            bufferCreateInfo.usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
            VmaAllocationCreateInfo allocationCreateInfo { };
            allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
            allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
            dvk_result(gvk::Buffer::create(mDevice, &bufferCreateInfo, &allocationCreateInfo, &mVertexIndexBuffer));
        }
        if (vertexIndexDataSize) {
            uint8_t* pVertexIndexData = nullptr;
            dvk_result(vmaMapMemory(mDevice.get<VmaAllocator>(), mVertexIndexBuffer.get<VmaAllocation>(), (void**)&pVertexIndexData));
            memcpy(pVertexIndexData, vertices.data(), vertexDataSize);
            memcpy(pVertexIndexData + vertexDataSize, indices.data(), indexDataSize);
            dvk_result(vmaFlushAllocation(mDevice.get<VmaAllocator>(), mVertexIndexBuffer.get<VmaAllocation>(), 0, vertexIndexDataSize));
            vmaUnmapMemory(mDevice.get<VmaAllocator>(), mVertexIndexBuffer.get<VmaAllocation>());
        }

        TextMeshUniforms uniforms { };
        uniforms.world = transform.world_from_local();
        uint8_t* pUniformData = nullptr;
        dvk_result(vmaMapMemory(mDevice.get<VmaAllocator>(), mUniformBuffer.get<VmaAllocation>(), (void**)&pUniformData));
        memcpy(pUniformData, &uniforms, sizeof(TextMeshUniforms));
        dvk_result(vmaFlushAllocation(mDevice.get<VmaAllocator>(), mUniformBuffer.get<VmaAllocation>(), 0, sizeof(TextMeshUniforms)));
        vmaUnmapMemory(mDevice.get<VmaAllocator>(), mUniformBuffer.get<VmaAllocation>());
    } dvk_result_scope_end;
    assert(vkResult == VK_SUCCESS);
}

void Renderer<dst::text::Mesh>::record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const Renderer<dst::text::Font>& fontRenderer)
{
    assert(commandBuffer);
    if (mIndexCount) {
        const auto& dispatchTable = mDevice.get<gvk::DispatchTable>();
        VkDeviceSize vertexDataOffset = 0;
        dispatchTable.gvkCmdBindVertexBuffers(commandBuffer, 0, 1, &mVertexIndexBuffer.get<const VkBuffer&>(), &vertexDataOffset);
        dispatchTable.gvkCmdBindIndexBuffer(commandBuffer, mVertexIndexBuffer, mIndexDataOffset, gvk::get_index_type<uint16_t>());
        dispatchTable.gvkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, fontRenderer.get_pipeline().get<gvk::PipelineLayout>(), 2, 1, &(const VkDescriptorSet&)mDescriptorSet, 0, nullptr);
        dispatchTable.gvkCmdDrawIndexed(commandBuffer, mIndexCount, 1, 0, 0, 0);
    }
}

VkResult Renderer<dst::text::Mesh>::create_uniform_buffer()
{
    assert(mDevice);
    auto bufferCreateInfo = gvk::get_default<VkBufferCreateInfo>();
    bufferCreateInfo.size = sizeof(TextMeshUniforms);
    bufferCreateInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    VmaAllocationCreateInfo vmaAllocationCreateInfo{ };
    vmaAllocationCreateInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT | VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    vmaAllocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    return gvk::Buffer::create(mDevice, &bufferCreateInfo, &vmaAllocationCreateInfo, &mUniformBuffer);
}

VkResult Renderer<dst::text::Mesh>::allocate_descriptor_set(const Renderer<dst::text::Font>& fontRenderer)
{
    assert(mDevice);
    return dvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        std::vector<gvk::DescriptorSet> descriptorSets;
        dvk_result(dst_sample_allocate_descriptor_sets_HACK(fontRenderer.get_pipeline(), descriptorSets));
        assert(descriptorSets.size() == 3);
        mDescriptorSet = descriptorSets[2];
        assert(mDescriptorSet);
    } dvk_result_scope_end;
}

void Renderer<dst::text::Mesh>::update_descriptor_set()
{
    auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
    descriptorBufferInfo.buffer = mUniformBuffer;
    auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
    writeDescriptorSet.dstSet = mDescriptorSet;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
    mDevice.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(mDevice, 1, &writeDescriptorSet, 0, nullptr);
}

} // namespace gfx
} // namespace dst
