
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

#include "shape-shooter/bloom.hpp"
#include "shape-shooter/utilities.hpp"

#include "imgui.h"

namespace shape_shooter {

struct BloomPushConstants
{
    glm::vec2 offset{ };
    float scale{ };
    float strength{ };
};

struct CombinePushConstants {
    float baseIntensity{ };
    float baseSaturation{ };
    float bloomIntensity{ };
    float bloomSaturation{ };
};

static VkResult create_bloom_pipeline(
    const gvk::RenderPass& renderPass,
    VkCullModeFlagBits cullMode,
    VkPolygonMode polygonMode,
    gvk::spirv::ShaderInfo& vertexShaderInfo,
    gvk::spirv::ShaderInfo& fragmentShaderInfo,
    gvk::Pipeline* pPipeline
)
{
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        // Create a gvk::spirv::Compiler, compile GLSL to SPIR-V, then validate both
        //  shaders.
        gvk::spirv::Context spirvContext;
        gvk_result(gvk::spirv::Context::create(&gvk::get_default<gvk::spirv::Context::CreateInfo>(), &spirvContext));
        spirvContext.compile(&vertexShaderInfo);
        gvk_result(dst_sample_validate_shader_info(vertexShaderInfo));
        spirvContext.compile(&fragmentShaderInfo);
        gvk_result(dst_sample_validate_shader_info(fragmentShaderInfo));
        auto vsVkResult = dst_sample_validate_shader_info(vertexShaderInfo);
        auto fsVkResult = dst_sample_validate_shader_info(fragmentShaderInfo);
        gvk_result(vsVkResult);
        gvk_result(fsVkResult);

        // Create a gvk::ShaderModule for the vertex shader.
        auto vertexShaderModuleCreateInfo = gvk::get_default<VkShaderModuleCreateInfo>();
        vertexShaderModuleCreateInfo.codeSize = vertexShaderInfo.bytecode.size() * sizeof(uint32_t);
        vertexShaderModuleCreateInfo.pCode = vertexShaderInfo.bytecode.data();
        gvk::ShaderModule vertexShaderModule;
        gvk_result(gvk::ShaderModule::create(renderPass.get<gvk::Device>(), &vertexShaderModuleCreateInfo, nullptr, &vertexShaderModule));
        auto vertexPipelineShaderStageCreateInfo = gvk::get_default<VkPipelineShaderStageCreateInfo>();
        vertexPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexPipelineShaderStageCreateInfo.module = vertexShaderModule;

        // Create a gvk::ShaderModule for the fragment shader.
        auto fragmentShaderModuleCreateInfo = gvk::get_default<VkShaderModuleCreateInfo>();
        fragmentShaderModuleCreateInfo.codeSize = fragmentShaderInfo.bytecode.size() * sizeof(uint32_t);
        fragmentShaderModuleCreateInfo.pCode = fragmentShaderInfo.bytecode.data();
        gvk::ShaderModule fragmentShaderModule;
        gvk_result(gvk::ShaderModule::create(renderPass.get<gvk::Device>(), &fragmentShaderModuleCreateInfo, nullptr, &fragmentShaderModule));
        auto fragmentPipelineShaderStageCreateInfo = gvk::get_default<VkPipelineShaderStageCreateInfo>();
        fragmentPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentPipelineShaderStageCreateInfo.module = fragmentShaderModule;

        // Create an array of VkPipelineShaderStageCreateInfos for the shaders used in
        //  this gvk::Pipeline.
        std::array<VkPipelineShaderStageCreateInfo, 2> pipelineShaderStageCreateInfos {
            vertexPipelineShaderStageCreateInfo,
            fragmentPipelineShaderStageCreateInfo,
        };

#if 0
        // VkVertexInputBindingDescription describes how the gvk::Pipeline should
        //  traverse vertex buffer data when draw calls are issued.
        // NOTE : gvk::get_vertex_description<VertexType>(0) is used to get an array of
        //  VkVertexInputAttributeDescriptions at binding 0 which indicates that the
        //  array is associated with the 0th element of pVertexBindingDescriptions.
        VkVertexInputBindingDescription vertexInputBindingDescription { 0, sizeof(VertexType), VK_VERTEX_INPUT_RATE_VERTEX };
        auto vertexInputAttributeDescriptions = gvk::get_vertex_description<VertexType>(0);
        auto pipelineVertexInputStateCreateInfo = gvk::get_default<VkPipelineVertexInputStateCreateInfo>();
        pipelineVertexInputStateCreateInfo.vertexBindingDescriptionCount = 1;
        pipelineVertexInputStateCreateInfo.pVertexBindingDescriptions = &vertexInputBindingDescription;
        pipelineVertexInputStateCreateInfo.vertexAttributeDescriptionCount = (uint32_t)vertexInputAttributeDescriptions.size();
        pipelineVertexInputStateCreateInfo.pVertexAttributeDescriptions = vertexInputAttributeDescriptions.data();
#endif

        // VkPipelineRasterizationStateCreateInfo describes how rasterization should
        //  occur...this includes parameters for polygon mode, winding order, face
        //  culling, etc.
        auto pipelineRasterizationStateCreateInfo = gvk::get_default<VkPipelineRasterizationStateCreateInfo>();
        pipelineRasterizationStateCreateInfo.cullMode = cullMode;
        pipelineRasterizationStateCreateInfo.polygonMode = polygonMode;

        // VkPipelineMultisampleStateCreateInfo describes how multi sampling should
        //  occur.  rasterizationSamples should match the sample count of the
        //  gvk::RenderPass objects that will be used with this gvk::Pipeline.
        auto pipelineMultisampleStateCreateInfo = gvk::get_default<VkPipelineMultisampleStateCreateInfo>();
        pipelineMultisampleStateCreateInfo.rasterizationSamples = dst_sample_get_render_pass_sample_count(renderPass);

        // VkPipelineDepthStencilStateCreateInfo describes how depth should be handled
        //  during fragment shading.
        auto depthTestEnable = dst_sample_get_render_pass_depth_format(renderPass) != VK_FORMAT_UNDEFINED;
        auto pipelineDepthStencilStateCreateInfo = gvk::get_default<VkPipelineDepthStencilStateCreateInfo>();
        pipelineDepthStencilStateCreateInfo.depthTestEnable = depthTestEnable;
        pipelineDepthStencilStateCreateInfo.depthWriteEnable = depthTestEnable;
        pipelineDepthStencilStateCreateInfo.depthCompareOp = VK_COMPARE_OP_LESS;

        // Populate a gvk::spirv::BindingInfo with our gvk::spirv::ShaderInfo objects.
        //  This will run the shader byte code through SPIRV-Cross to reflect the
        //  resource bindings used in the shaders.  When the gvk::spirv::BindingInfo is
        //  passed to create_pipeline_layout() it will be used to create the necessary
        //  gvk::DescriptorSetLayout objects for the gvk::PipelineLayout.
        gvk::spirv::BindingInfo spirvBindingInfo;
        spirvBindingInfo.add_shader(vertexShaderInfo);
        spirvBindingInfo.add_shader(fragmentShaderInfo);
        gvk::PipelineLayout pipelineLayout;
        gvk_result(gvk::spirv::create_pipeline_layout(renderPass.get<gvk::Device>(), spirvBindingInfo, nullptr, &pipelineLayout));

        // Finally we populate a VkGraphicsPipelineCreateInfo with the components
        //  necessary for this gvk::Pipeline.
        // NOTE : gvk::get_default<VkGraphicsPipelineCreateInfo>() is used to get the
        //  VkGraphicsPipelineCreateInfo that is prepared for this gvk::Pipeline.
        //  gvk::get_default<>() automatically sets sTypes and sensible default values
        //  where appropriate, for gvk::get_default<VkGraphicsPipelineCreateInfo>() the
        //  default values are generally configured to noop so that only portions of
        //  gvk::Pipeline that are purposefully configured will have an effect.
        auto graphicsPipelineCreateInfo = gvk::get_default<VkGraphicsPipelineCreateInfo>();
        graphicsPipelineCreateInfo.stageCount = (uint32_t)pipelineShaderStageCreateInfos.size();
        graphicsPipelineCreateInfo.pStages = pipelineShaderStageCreateInfos.data();
#if 0
        if (pipelineVertexInputStateCreateInfo.vertexAttributeDescriptionCount) {
            graphicsPipelineCreateInfo.pVertexInputState = &pipelineVertexInputStateCreateInfo;
        }
#endif
        graphicsPipelineCreateInfo.pRasterizationState = &pipelineRasterizationStateCreateInfo;
        graphicsPipelineCreateInfo.pMultisampleState = &pipelineMultisampleStateCreateInfo;
        graphicsPipelineCreateInfo.pDepthStencilState = &pipelineDepthStencilStateCreateInfo;
        graphicsPipelineCreateInfo.layout = pipelineLayout;
        graphicsPipelineCreateInfo.renderPass = renderPass;
        gvk_result(gvk::Pipeline::create(renderPass.get<gvk::Device>(), VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, nullptr, pPipeline));
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult BloomRenderer::create(const gvk::Context& gvkContext, const CreateInfo* pCreateInfo, BloomRenderer* pBloom)
{
    (void)gvkContext;
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(pCreateInfo ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        gvk_result(pCreateInfo->renderPass ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        gvk_result(pBloom ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);

        // TODO : Documentation
        DstSampleRenderTargetCreateInfo dstSampleRenderTargetCreateInfo{ };
        dstSampleRenderTargetCreateInfo.extent = { 16, 16 };
        dstSampleRenderTargetCreateInfo.colorFormat = pCreateInfo->format;
        gvk_result(dst_sample_create_render_target(gvkContext, dstSampleRenderTargetCreateInfo, &pBloom->mRenderTargets[0]));

        // TODO : Documentation
        gvk::spirv::ShaderInfo vertexShaderInfo {
            .language = gvk::spirv::ShadingLanguage::Glsl,
            .stage = VK_SHADER_STAGE_VERTEX_BIT,
            .lineOffset = __LINE__,
            .source = R"(
                #version 450

                layout(location = 0) out vec2 fsTexcoord;
                out gl_PerVertex { vec4 gl_Position; };

                void main()
                {
                    fsTexcoord = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
                    gl_Position = vec4(fsTexcoord * 2 - 1, 0, 1);
                }
            )"
        };

        // TODO : Documentation
        gvk::spirv::ShaderInfo extractFragmentShaderInfo {
            .language = gvk::spirv::ShadingLanguage::Glsl,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .lineOffset = __LINE__,
            .source = R"(
                #version 450

                layout(push_constant) uniform PushConstants {
                    float threshold;
                } pc;

                layout(set = 0, binding = 0) uniform sampler2D image;
                layout(location = 0) in vec2 fsTexcoord;
                layout(location = 0) out vec4 fragColor;

                void main()
                {
                    fragColor = texture(image, fsTexcoord);
                    fragColor = clamp((fragColor - pc.threshold) / (1 - pc.threshold), 0, 1);
                }
            )"
        };

        // TODO : Documentation
        gvk::spirv::ShaderInfo blurFragmentShaderInfo{
            .language = gvk::spirv::ShadingLanguage::Glsl,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .lineOffset = __LINE__,
            .source = R"(
                #version 450

                layout(push_constant) uniform PushConstants {
                    vec2 offset;
                    float scale;
                    float strength;
                } pc;

                layout(set = 0, binding = 0) uniform sampler2D image;
                layout(location = 0) in vec2 fsTexcoord;
                layout(location = 0) out vec4 fragColor;

                void main()
                {
                    float weight[5];
                    weight[0] = 0.227027;
                    weight[1] = 0.1945946;
                    weight[2] = 0.1216216;
                    weight[3] = 0.054054;
                    weight[4] = 0.016216;

                    fragColor = vec4(0);
                    vec2 offset = 1.0 / textureSize(image, 0) * pc.offset * pc.scale;
                    vec3 result = texture(image, fsTexcoord).rgb * weight[0];
                    for (int i = 0; i < 5; ++i) {
                        fragColor += texture(image, fsTexcoord + offset * i) * weight[i] * pc.strength;
                        fragColor += texture(image, fsTexcoord - offset * i) * weight[i] * pc.strength;
                    }
                    fragColor.a = 1;
                }
            )"
        };

        // TODO : Documentation
        gvk::spirv::ShaderInfo combineFragmentShaderInfo{
            .language = gvk::spirv::ShadingLanguage::Glsl,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .lineOffset = __LINE__,
            .source = R"(
                #version 450

                layout(push_constant) uniform PushConstants {
                    float baseIntensity;
                    float baseSaturation;
                    float bloomIntensity;
                    float bloomSaturation;
                } pc;

                layout(set = 0, binding = 0) uniform sampler2D baseImage;
                layout(set = 0, binding = 1) uniform sampler2D bloomImage;
                layout(location = 0) in vec2 fsTexcoord;
                layout(location = 0) out vec4 fragColor;

                // Helper for modifying the saturation of a color.
                vec4 adjust_saturation(vec4 color, float saturation)
                {
                    // The constants 0.3, 0.59, and 0.11 are chosen because the
                    // human eye is more sensitive to green light, and less to blue.
                    float grey = dot(color.rgb, vec3(0.3, 0.59, 0.11));
                    return mix(vec4(grey), color, saturation);
                }

                void main()
                {
                    // Look up the bloom and original base image colors.
                    vec4 baseColor = texture(baseImage, fsTexcoord);
                    vec4 bloomColor = texture(bloomImage, fsTexcoord);

                    // Adjust color saturation and intensity.
                    baseColor = adjust_saturation(baseColor, pc.baseSaturation) * pc.baseIntensity;
                    bloomColor = adjust_saturation(bloomColor, pc.bloomSaturation) * pc.bloomIntensity;

                    // Darken down the base image in areas where there is a lot of bloom,
                    // to prevent things looking excessively burned-out.
                    baseColor *= (1 - clamp(bloomColor, 0, 1));

                    // Combine the two images.
                    fragColor = baseColor + bloomColor;
                }
            )"
        };

        // TODO : Documentation
        gvk::spirv::ShaderInfo blitFragmentShaderInfo{
            .language = gvk::spirv::ShadingLanguage::Glsl,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .lineOffset = __LINE__,
            .source = R"(
                #version 450

                layout(set = 0, binding = 0) uniform sampler2D image;

                layout(location = 0) in vec2 inTexCoord;
                layout(location = 0) out vec4 outColor;

                void main()
                {
                    outColor = texture(image, inTexCoord);
                }
            )"
        };

        // TODO : Documentation
        auto renderPass = pBloom->mRenderTargets[0].get<gvk::Framebuffer>().get<gvk::RenderPass>();
        gvk_result(create_bloom_pipeline(renderPass, VK_CULL_MODE_BACK_BIT, VK_POLYGON_MODE_FILL, vertexShaderInfo, extractFragmentShaderInfo, &pBloom->mExtractPipeline));
        gvk_result(create_bloom_pipeline(renderPass, VK_CULL_MODE_BACK_BIT, VK_POLYGON_MODE_FILL, vertexShaderInfo, blurFragmentShaderInfo, &pBloom->mBlurPipeline));
        gvk_result(create_bloom_pipeline(renderPass, VK_CULL_MODE_BACK_BIT, VK_POLYGON_MODE_FILL, vertexShaderInfo, combineFragmentShaderInfo, &pBloom->mCombinePipeline));
        gvk_result(create_bloom_pipeline(pCreateInfo->renderPass, VK_CULL_MODE_BACK_BIT, VK_POLYGON_MODE_FILL, vertexShaderInfo, blitFragmentShaderInfo, &pBloom->mBlitPipeline));

        // TODO : Documentation
        std::vector<gvk::DescriptorSet> descriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets(pBloom->mExtractPipeline, descriptorSets));
        gvk_result(descriptorSets.size() == 1 ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        pBloom->mExtractDescriptorSet = descriptorSets[0];

        // TODO : Documentation
        gvk_result(dst_sample_allocate_descriptor_sets(pBloom->mBlurPipeline, descriptorSets));
        gvk_result(descriptorSets.size() == 1 ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        pBloom->mBlurDescriptorSet = descriptorSets[0];

        // TODO : Documentation
        gvk_result(dst_sample_allocate_descriptor_sets(pBloom->mCombinePipeline, descriptorSets));
        gvk_result(descriptorSets.size() == 1 ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        pBloom->mCombineDescriptorSet = descriptorSets[0];

        // TODO : Documentation
        gvk_result(dst_sample_allocate_descriptor_sets(pBloom->mBlitPipeline, descriptorSets));
        gvk_result(descriptorSets.size() == 1 ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        pBloom->mBlitDescriptorSet = descriptorSets[0];

        // TODO : Documentation
        gvk_result(gvk::Sampler::create(renderPass.get<gvk::Device>(), &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &pBloom->mSampler));
    } gvk_result_scope_end;
    return gvkResult;
}

#if 0
VkResult BloomRenderer::begin_render_pass(const gvk::CommandBuffer& commandBuffer, const gvk::RenderTarget& inputRenderTarget)
{
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(inputRenderTarget ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        mOutputRenderTarget = inputRenderTarget;
        auto inputFramebufferCreateInfo = mInputRenderTarget ? mInputRenderTarget.get<VkFramebufferCreateInfo>() : gvk::get_default<VkFramebufferCreateInfo>();
        auto outputFramebufferCreateInfo = mOutputRenderTarget ? mOutputRenderTarget.get<VkFramebufferCreateInfo>() : gvk::get_default<VkFramebufferCreateInfo>();
        if (inputFramebufferCreateInfo.width != outputFramebufferCreateInfo.width || inputFramebufferCreateInfo.height != outputFramebufferCreateInfo.height) {
            gvk::Auto<VkFramebufferCreateInfo> autoFramebufferCreateInfo = inputRenderTarget.get<VkFramebufferCreateInfo>();
            gvk::detail::enumerate_structure_handles(
                *autoFramebufferCreateInfo,
                [](VkObjectType, const uint64_t& handle)
                {
                    const_cast<uint64_t&>(handle) = 0;
                }
            );
            gvk_result(gvk::RenderTarget::create(inputRenderTarget.get<gvk::Device>(), &*autoFramebufferCreateInfo, nullptr, &mInputRenderTarget));
        }
        commandBuffer.CmdBeginRenderPass(&mInputRenderTarget.get<VkRenderPassBeginInfo>(), VK_SUBPASS_CONTENTS_INLINE);
    } gvk_result_scope_end;
    return gvkResult;
}

void BloomRenderer::end_render_pass(const gvk::CommandBuffer& commandBuffer)
{
    commandBuffer.CmdEndRenderPass();

    // Extract
    commandBuffer.CmdBeginRenderPass(nullptr, VK_SUBPASS_CONTENTS_INLINE);
    commandBuffer.CmdEndRenderPass();

    // Horizontal blur
    commandBuffer.CmdBeginRenderPass(nullptr, VK_SUBPASS_CONTENTS_INLINE);
    commandBuffer.CmdEndRenderPass();

    // Vertical blur
    commandBuffer.CmdBeginRenderPass(nullptr, VK_SUBPASS_CONTENTS_INLINE);
    commandBuffer.CmdEndRenderPass();
}
#endif

VkResult BloomRenderer::record_cmds(const Settings& settings, const gvk::Context& gvkContext, const gvk::CommandBuffer& commandBuffer, VkFormat outputColorFormat, const gvk::RenderTarget& inputRenderTarget)
{
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(inputRenderTarget ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED);
        auto framebufferCreateInfo = inputRenderTarget.get<VkFramebufferCreateInfo>();
        if (!mRenderTargets[0] || !mRenderTargets[1] ||
            mRenderTargets[0].get<VkFramebufferCreateInfo>().width != framebufferCreateInfo.width ||
            mRenderTargets[0].get<VkFramebufferCreateInfo>().height != framebufferCreateInfo.height) {
            auto dstSampleRenderTargetCreateInfo = gvk::get_default<DstSampleRenderTargetCreateInfo>();
            dstSampleRenderTargetCreateInfo.extent = { framebufferCreateInfo.width, framebufferCreateInfo.height };
            dstSampleRenderTargetCreateInfo.colorFormat = outputColorFormat;
            for (auto& renderTarget : mRenderTargets) {
                gvk_result(dst_sample_create_render_target(gvkContext, dstSampleRenderTargetCreateInfo, &renderTarget));
            }
        }

        // TODO : Documentation
        auto descriptorImageInfo = gvk::get_default<VkDescriptorImageInfo>();
        descriptorImageInfo.sampler = mSampler;
        // TODO : Currently assuming that Image[1] is the MSAA resolve target...needs to
        //  be automatically inferred from the RenderTarget.
        descriptorImageInfo.imageView = inputRenderTarget.get<gvk::Framebuffer>().get<gvk::ImageViews>()[1];
        descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
        writeDescriptorSet.dstSet = mExtractDescriptorSet;
        writeDescriptorSet.descriptorCount = 1;
        writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writeDescriptorSet.pImageInfo = &descriptorImageInfo;
        commandBuffer.get<gvk::Device>().UpdateDescriptorSets(1, &writeDescriptorSet, 0, nullptr);

        // Extract
        auto renderPassBeginInfo = mRenderTargets[0].get<VkRenderPassBeginInfo>();
        commandBuffer.CmdBeginRenderPass(&renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
        {
            VkRect2D scissor{ { }, renderPassBeginInfo.renderArea.extent };
            commandBuffer.CmdSetScissor(0, 1, &scissor);
            VkViewport viewport{ 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
            commandBuffer.CmdSetViewport(0, 1, &viewport);
            commandBuffer.CmdBindPipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, mExtractPipeline);
            commandBuffer.CmdBindDescriptorSets(VK_PIPELINE_BIND_POINT_GRAPHICS, mExtractPipeline.get<gvk::PipelineLayout>(), 0, 1, &mExtractDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
            commandBuffer.CmdPushConstants(mExtractPipeline.get<gvk::PipelineLayout>(), VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(float), &settings.threshold);
            commandBuffer.CmdDraw(3, 1, 0, 0);
        }
        commandBuffer.CmdEndRenderPass();

#if 1
        // TODO : Documentation
        descriptorImageInfo.imageView = mRenderTargets[0].get<gvk::Framebuffer>().get<gvk::ImageViews>()[0];
        writeDescriptorSet.dstSet = mBlurDescriptorSet;
        commandBuffer.get<gvk::Device>().UpdateDescriptorSets(1, &writeDescriptorSet, 0, nullptr);

        // Blur
        commandBuffer.CmdBeginRenderPass(&mRenderTargets[1].get<VkRenderPassBeginInfo>(), VK_SUBPASS_CONTENTS_INLINE);
        {
            commandBuffer.CmdBindPipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, mBlurPipeline);
            commandBuffer.CmdBindDescriptorSets(VK_PIPELINE_BIND_POINT_GRAPHICS, mBlurPipeline.get<gvk::PipelineLayout>(), 0, 1, &mBlurDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
            BloomPushConstants bloomPushConstants{ };
            bloomPushConstants.scale = settings.blurScale;
            bloomPushConstants.strength = settings.blurScale;
            // H
            bloomPushConstants.offset = { 1, 0 };
            commandBuffer.CmdPushConstants(mBlurPipeline.get<gvk::PipelineLayout>(), VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(BloomPushConstants), &bloomPushConstants);
            commandBuffer.CmdDraw(3, 1, 0, 0);
            // V
            bloomPushConstants.offset = { 0, 1 };
            commandBuffer.CmdPushConstants(mBlurPipeline.get<gvk::PipelineLayout>(), VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(BloomPushConstants), &bloomPushConstants);
            commandBuffer.CmdDraw(3, 1, 0, 0);
        }
        commandBuffer.CmdEndRenderPass();

        // TODO : Documentation
        std::array<VkDescriptorImageInfo, 2> descriptorImageInfos{ gvk::get_default<VkDescriptorImageInfo>(), gvk::get_default<VkDescriptorImageInfo>() };
        descriptorImageInfos[0].sampler = mSampler;
        descriptorImageInfos[0].imageView = inputRenderTarget.get<gvk::Framebuffer>().get<gvk::ImageViews>()[1];
        descriptorImageInfos[0].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        descriptorImageInfos[1].sampler = mSampler;
        descriptorImageInfos[1].imageView = mRenderTargets[1].get<gvk::Framebuffer>().get<gvk::ImageViews>()[0];
        descriptorImageInfos[1].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        writeDescriptorSet.dstSet = mCombineDescriptorSet;
        writeDescriptorSet.descriptorCount = 2;
        writeDescriptorSet.pImageInfo = descriptorImageInfos.data();
        commandBuffer.get<gvk::Device>().UpdateDescriptorSets(1, &writeDescriptorSet, 0, nullptr);

        // Combine
        commandBuffer.CmdBeginRenderPass(&mRenderTargets[0].get<VkRenderPassBeginInfo>(), VK_SUBPASS_CONTENTS_INLINE);
        {
            commandBuffer.CmdBindPipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, mCombinePipeline);
            commandBuffer.CmdBindDescriptorSets(VK_PIPELINE_BIND_POINT_GRAPHICS, mCombinePipeline.get<gvk::PipelineLayout>(), 0, 1, &mCombineDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
            CombinePushConstants combinePushConstants{ };
            combinePushConstants.baseIntensity = settings.baseIntensity;
            combinePushConstants.baseSaturation = settings.baseSaturation;
            combinePushConstants.bloomIntensity = settings.bloomIntensity;
            combinePushConstants.bloomSaturation = settings.bloomSaturation;
            commandBuffer.CmdPushConstants(mCombinePipeline.get<gvk::PipelineLayout>(), VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(CombinePushConstants), &combinePushConstants);
            commandBuffer.CmdDraw(3, 1, 0, 0);
        }
        commandBuffer.CmdEndRenderPass();
#endif
    } gvk_result_scope_end;
    return gvkResult;
}

void BloomRenderer::draw_render_target(const Settings& settings, const gvk::CommandBuffer& commandBuffer)
{
    (void)settings;
    // TODO : Documentation
    auto descriptorImageInfo = gvk::get_default<VkDescriptorImageInfo>();
    descriptorImageInfo.sampler = mSampler;
    descriptorImageInfo.imageView = mRenderTargets[0].get<gvk::Framebuffer>().get<gvk::ImageViews>()[0];
    descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
    writeDescriptorSet.dstSet = mBlitDescriptorSet;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet.pImageInfo = &descriptorImageInfo;
    commandBuffer.get<gvk::Device>().UpdateDescriptorSets(1, &writeDescriptorSet, 0, nullptr);
    commandBuffer.CmdBindPipeline(VK_PIPELINE_BIND_POINT_GRAPHICS, mBlitPipeline);
    commandBuffer.CmdBindDescriptorSets(VK_PIPELINE_BIND_POINT_GRAPHICS, mBlitPipeline.get<gvk::PipelineLayout>(), 0, 1, &mBlitDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
    commandBuffer.CmdDraw(3, 1, 0, 0);
}

void BloomRenderer::on_gui(Settings& settings)
{
    if (ImGui::CollapsingHeader("Bloom")) {
        ImGui::Indent();
        {
            ImGui::Checkbox("Enabled", &settings.enabled);
            ImGui::BeginDisabled(!settings.enabled);
            ImGui::DragFloat("Threshold", &settings.threshold, 0.01f, 0.0f, 1.0f);
            ImGui::DragFloat("Blur Scale", &settings.blurScale, 0.01f, 0.0f, 8.0f);
            ImGui::DragFloat("Blur Strength", &settings.blurStrength, 0.01f, 0.0f, 8.0f);
            ImGui::DragFloat("Base Intensity", &settings.baseIntensity, 0.01f, 0.0f, 8.0f);
            ImGui::DragFloat("Bloom Intensity", &settings.bloomIntensity, 0.01f, 0.0f, 8.0f);
            ImGui::DragFloat("Base Saturation", &settings.baseSaturation, 0.01f, 0.0f, 8.0f);
            ImGui::DragFloat("Bloom Saturation", &settings.bloomSaturation, 0.01f, 0.0f, 8.0f);
            ImGui::EndDisabled();
        }
        ImGui::Unindent();
    }
}

} // namespace shape_shooter
