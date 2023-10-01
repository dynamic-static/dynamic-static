
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

#include "dynamic-static.graphics/renderer.hpp"
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

VkResult LineRenderer::create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, const CreateInfo& createInfo, LineRenderer* pRenderer)
{
    (void)createInfo;
    assert(pRenderer);
    pRenderer->reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(pRenderer->create_pipeline(gvkContext, renderPass));
        gvk_result(pRenderer->allocate_descriptor_set(gvkContext));
    } gvk_result_scope_end;
    return gvkResult;
}

LineRenderer::~LineRenderer()
{
    reset();
}

void LineRenderer::reset()
{
    mPipeline.reset();
    mStorageBuffer.reset();
    mDescriptorSet.reset();
}

void LineRenderer::begin_line_batch()
{
    mPoints.clear();
}

void LineRenderer::submit(uint32_t pointCount, const Point* pPoints)
{
    if (pointCount && pPoints) {
        mPoints.insert(mPoints.end(), pPoints[0]);
        mPoints.insert(mPoints.end(), pPoints, pPoints + pointCount);
        mPoints.insert(mPoints.end(), pPoints[pointCount - 1]);
    }
}

void LineRenderer::end_line_batch()
{
    assert(mPipeline);
    if (!mPoints.empty()) {
        gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
            const auto& device = mPipeline.get<gvk::Device>();
            auto size = (VkDeviceSize)(mPoints.size() * sizeof(Point));
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

                std::array<VkWriteDescriptorSet, 1> writeDescriptorSets {
                    gvk::get_default<VkWriteDescriptorSet>(),
                    // gvk::get_default<VkWriteDescriptorSet>(),
                };
                // Storage buffer
                auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
                descriptorBufferInfo.buffer = mStorageBuffer;
                writeDescriptorSets[0].dstSet = mDescriptorSet;
                writeDescriptorSets[0].dstBinding = 0;
                writeDescriptorSets[0].descriptorCount = 1;
                writeDescriptorSets[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                writeDescriptorSets[0].pBufferInfo = &descriptorBufferInfo;
                // // Image array
                // std::vector<VkDescriptorImageInfo> descriptorImageInfos;
                // descriptorImageInfos.reserve(mImages.size());
                // for (const auto& image : mImages) {
                //     auto descriptorImageInfo = gvk::get_default<VkDescriptorImageInfo>();
                //     descriptorImageInfo.sampler = mSampler;
                //     descriptorImageInfo.imageView = image.second;
                //     descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                //     descriptorImageInfos.push_back(descriptorImageInfo);
                // }
                // writeDescriptorSets[1].dstSet = mDescriptorSet;
                // writeDescriptorSets[1].dstBinding = 1;
                // writeDescriptorSets[1].descriptorCount = (uint32_t)mImages.size();
                // writeDescriptorSets[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                // writeDescriptorSets[1].pImageInfo = descriptorImageInfos.data();
                device.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(device, (uint32_t)writeDescriptorSets.size(), writeDescriptorSets.data(), 0, nullptr);
            }

            // Update VkBuffer
            auto vmaAllocator = device.get<VmaAllocator>();
            auto vmaAllocation = mStorageBuffer.get<VmaAllocation>();
            uint8_t* pData = nullptr;
            gvk_result(vmaMapMemory(vmaAllocator, vmaAllocation, (void**)&pData));
            memcpy(pData, mPoints.data(), size);
            gvk_result(vmaFlushAllocation(vmaAllocator, vmaAllocation, 0, size));
            vmaUnmapMemory(vmaAllocator, vmaAllocation);
        } gvk_result_scope_end;
        assert(gvkResult == VK_SUCCESS);
    }
}

void LineRenderer::record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera, const glm::vec2& resolution) const
{
    if (!mPoints.empty()) {
        auto bindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        const auto& pipelineLayout = mPipeline.get<gvk::PipelineLayout>();
        const auto& dispatchTable = commandBuffer.get<gvk::Device>().get<gvk::DispatchTable>();
        dispatchTable.gvkCmdBindPipeline(commandBuffer, bindPoint, mPipeline);
        struct PushConstants
        {
            glm::mat4 view;
            glm::mat4 projection;
            glm::vec2 resolution;
        } pushConstants;
        pushConstants.view = camera.view();
        pushConstants.projection = camera.projection();
        pushConstants.resolution = resolution;
        dispatchTable.gvkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pushConstants), &pushConstants);
        dispatchTable.gvkCmdBindDescriptorSets(commandBuffer, bindPoint, pipelineLayout, 0, 1, &mDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
        dispatchTable.gvkCmdDraw(commandBuffer, 4, (uint32_t)mPoints.size() - 2, 0, 1);
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

VkResult LineRenderer::create_pipeline(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass)
{
    assert(gvkContext);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        auto vertexShaderInfo = gvk::get_default<gvk::spirv::ShaderInfo>();
        vertexShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        vertexShaderInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderInfo.lineOffset = __LINE__;
        vertexShaderInfo.source = R"(
            #version 460

            struct Point
            {
                vec4 position;
                vec4 color;
                vec4 width;
            };

            vec4 Vertices[4] = vec4[](
                vec4(0, 0.5, 0, 1),       vec4(1,  0.5, 0, 1),
                /*
                                 +--------+
                                 |        |
                                 |        |
                                 |        |
                                 +--------+
                */
                vec4(0, -0.5, 0, 1),      vec4(1, -0.5, 0, 1)
            );

            layout(push_constant) uniform Camera
            {
                mat4 view;
                mat4 projection;
                vec2 resolution;
            } camera;

            layout(std140, set = 0, binding = 0) readonly buffer PointBuffer
            {
                Point points[];
            } pointBuffer;

            layout(location = 0) out vec4 fsColor;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };

            void main()
            {
                ///////////////////////////////////////////////////////////////////////////////
                // Point point = pointBuffer.points[gl_InstanceIndex];
                // vec4 position = point.position + Vertices[gl_VertexIndex];
                // gl_Position = camera.projection * camera.view * position;
                // fsColor = point.color;
                ///////////////////////////////////////////////////////////////////////////////

                ///////////////////////////////////////////////////////////////////////////////
                // Point point = pointBuffer.points[gl_InstanceIndex];
                // Point next = pointBuffer.points[gl_InstanceIndex + 1];
                // 
                // // vec4 p0 = camera.projection * camera.view * point.position;
                // // vec4 p1 = camera.projection * camera.view * next.position;
                // vec4 p0 = point.position;
                // vec4 p1 = next.position;
                // 
                // vec3 segment = p1.xyz - p0.xyz;
                // vec3 normal = normalize(cross(segment, vec3(0, 0, 1)));
                // vec4 vertices[4] = vec4[](
                //     vec4(p0.xyz + normal * point.width.r, 1),
                //     vec4(p0.xyz - normal * point.width.r, 1),
                //     vec4(p1.xyz + normal * next.width.r,  1),
                //     vec4(p1.xyz - normal * next.width.r,  1)
                // );
                // vec4 colors[4] = vec4[](
                //     point.color,
                //     point.color,
                //     next.color,
                //     next.color
                // );
                // gl_Position = camera.projection * camera.view * vertices[gl_VertexIndex];
                // fsColor = colors[gl_VertexIndex];
                ///////////////////////////////////////////////////////////////////////////////

                ///////////////////////////////////////////////////////////////////////////////
                // Point point = pointBuffer.points[gl_InstanceIndex];
                // Point next = pointBuffer.points[gl_InstanceIndex + 1];
                // vec4 p0 = camera.projection * camera.view * point.position;
                // vec4 p1 = camera.projection * camera.view * next.position;
                // vec2 segment = p1.xy - p0.xy;
                // vec2 perpendicular = normalize(vec2(segment.y, -segment.x));
                // vec4 vertices[4] = vec4[](
                //     p0 + vec4(-0.1 / camera.resolution.x,  0.1 / camera.resolution.y, 0, 1),
                //     p0 + vec4( 0.1 / camera.resolution.x,  0.1 / camera.resolution.y, 0, 1),
                //     p0 + vec4(-0.1 / camera.resolution.x, -0.1 / camera.resolution.y, 0, 1),
                //     p0 + vec4( 0.1 / camera.resolution.x, -0.1 / camera.resolution.y, 0, 1)
                // );
                // // vertices[0].xy += perpendicular * point.width.r;
                // // vertices[1].xy += perpendicular * next.width.r;
                // // vertices[2].xy -= perpendicular * point.width.r;
                // // vertices[3].xy -= perpendicular * next.width.r;
                // vec4 colors[4] = vec4[](
                //     point.color,
                //     next.color,
                //     point.color,
                //     next.color
                // );
                // gl_Position = vertices[gl_VertexIndex];
                // fsColor = colors[gl_VertexIndex];
                ///////////////////////////////////////////////////////////////////////////////

                ///////////////////////////////////////////////////////////////////////////////
                Point point0 = pointBuffer.points[gl_InstanceIndex];
                Point point1 = pointBuffer.points[gl_InstanceIndex + 1];
                vec4 p0 = point0.position;
                vec4 p1 = point1.position;
                vec2 segment = p1.xy - p0.xy;
                vec2 perpendicular = normalize(vec2(segment.y, -segment.x));
                vec4 vertices[4] = vec4[](
                    vec4(p0.xy - perpendicular * point0.width.r, 0, 1),
                    vec4(p1.xy - perpendicular * point1.width.r, 0, 1),
                    vec4(p0.xy + perpendicular * point0.width.r, 0, 1),
                    vec4(p1.xy + perpendicular * point1.width.r, 0, 1)
                );
                gl_Position = camera.projection * camera.view * vertices[gl_VertexIndex];
                vec4 colors[4] = vec4[](
                    point0.color,
                    point1.color,
                    point0.color,
                    point1.color
                );
                vec4 color = vec4(camera.resolution, 0, 0);
                color *= 0.00000000000000000000000000001;
                fsColor = colors[gl_VertexIndex] + color;
                ///////////////////////////////////////////////////////////////////////////////
            }
        )";

        auto fragmentShaderInfo = gvk::get_default<gvk::spirv::ShaderInfo>();
        fragmentShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        fragmentShaderInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderInfo.lineOffset = __LINE__;
        fragmentShaderInfo.source = R"(
            #version 450

            layout(location = 0) in vec4 fsColor;

            layout(location = 0) out vec4 fragColor;

            void main()
            {
                fragColor = fsColor;
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

        // // TODO : It would be good to figure out how to get this info from SPIRV-Cross
        // std::array<VkDescriptorBindingFlags, 2> descriptorBindingFlags { 0, VK_DESCRIPTOR_BINDING_VARIABLE_DESCRIPTOR_COUNT_BIT };
        // auto descriptorSetLayoutBindingFlagsCreateInfo = gvk::get_default<VkDescriptorSetLayoutBindingFlagsCreateInfo>();
        // descriptorSetLayoutBindingFlagsCreateInfo.bindingCount = (uint32_t)descriptorBindingFlags.size();
        // descriptorSetLayoutBindingFlagsCreateInfo.pBindingFlags = descriptorBindingFlags.data();
        // auto descriptorSetLayoutCreateInfo = gvk::get_default<VkDescriptorSetLayoutCreateInfo>();
        // descriptorSetLayoutCreateInfo.pNext = &descriptorSetLayoutBindingFlagsCreateInfo;
        // spirvBindingInfo.descriptorSetLayoutCreateInfos[0] = descriptorSetLayoutCreateInfo;
        // spirvBindingInfo.descriptorSetLayoutBindings[0][1].descriptorCount = (uint32_t)mImages.size();

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

VkResult LineRenderer::allocate_descriptor_set(const gvk::Context& gvkContext)
{
    (void)gvkContext;
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        std::vector<gvk::DescriptorSet> descriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets_HACK(mPipeline, 1, descriptorSets));
        assert(descriptorSets.size() == 1);
        mDescriptorSet = descriptorSets[0];
    } gvk_result_scope_end;
    return gvkResult;
}

} // namespace gfx
} // namespace dst
