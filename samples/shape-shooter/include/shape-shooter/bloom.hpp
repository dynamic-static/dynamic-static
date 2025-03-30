
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

#pragma once

#include "../../dynamic-static.sample-utilities.hpp"

namespace shape_shooter {

class BloomRenderer final
{
public:
    class Settings final
    {
    public:
        bool enabled{ true };
        float threshold{ 0.5f };
        float blurScale{ 1.0f };
        float blurStrength{ 1.5f };
        float baseIntensity{ 0.5f };
        float bloomIntensity{ 0.5f };
        float baseSaturation{ 0.5f };
        float bloomSaturation{ 0.5f };
    };

    class CreateInfo final
    {
    public:
        VkFormat format{ };
        gvk::RenderPass renderPass;
        Settings settings{ };
    };

    static VkResult create(const gvk::Context& gvkContext, const CreateInfo* pCreateInfo, BloomRenderer* pBloom);
    VkResult begin_render_pass(const gvk::CommandBuffer& commandBuffer, const gvk::RenderTarget& renderTarget);
    void end_render_pass(const gvk::CommandBuffer& commandBuffer);

    VkResult record_cmds(const Settings& settings, const gvk::Context& gvkContext, const gvk::CommandBuffer& commandBuffer, VkFormat outputColorFormat, const gvk::RenderTarget& inputRenderTarget);
    void draw_render_target(const Settings& settings, const gvk::CommandBuffer& commandBuffer);

    void on_gui(Settings& settings);

private:
    gvk::Pipeline mExtractPipeline;
    gvk::DescriptorSet mExtractDescriptorSet;
    gvk::Pipeline mBlurPipeline;
    gvk::DescriptorSet mBlurDescriptorSet;
    gvk::Pipeline mCombinePipeline;
    gvk::DescriptorSet mCombineDescriptorSet;
    gvk::Pipeline mBlitPipeline;
    gvk::DescriptorSet mBlitDescriptorSet;
    std::array<gvk::RenderTarget, 2> mRenderTargets;
    gvk::Sampler mSampler;
};

} // namespace shape_shooter
