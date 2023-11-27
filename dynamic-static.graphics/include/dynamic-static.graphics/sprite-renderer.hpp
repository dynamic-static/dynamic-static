
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

#include "dynamic-static.graphics/defines.hpp"

#include "gvk-math.hpp"

#include <set>
#include <vector>

namespace dst {
namespace gfx {

class SpriteRenderer final
{
public:
    class CreateInfo final
    {
    public:
        gvk::RenderPass renderPass;
        uint32_t imageCount{ };
        const gvk::ImageView* pImages{ };
    };

    static VkResult create(const gvk::Context& gvkContext, const CreateInfo& createInfo, SpriteRenderer* pSpriteRenderer);
    ~SpriteRenderer();
    void reset();

    const std::vector<gvk::ImageView>& get_images() const;
    const gvk::Pipeline& get_pipeline() const;
    void begin_sprite_batch();
    void submit(uint32_t imageIndex, const gvk::math::Transform& transform = { }, const glm::vec4& color = gvk::math::Color::White, const glm::vec4& uv = { 0, 0, 1, 1 });
    void submit(uint32_t imageIndex, const glm::vec3& translation, const glm::quat& rotation = { 1, 0, 0, 0 }, const glm::vec3& scale = { 1, 1, 1 }, const glm::vec4 & color = gvk::math::Color::White, const glm::vec4 & uv = { 0, 0, 1, 1 });
    void end_sprite_batch();
    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera) const;

private:
    struct Sprite
    {
        glm::vec4 extent{ };
        glm::vec4 uvMin{ };
        glm::vec4 uvMax{ };
        glm::vec4 color{ };
        glm::mat4 model{ };
    };

    VkResult create_pipeline(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass);
    VkResult allocate_descriptor_set(const gvk::Context& gvkContext);

    gvk::Pipeline mPipeline;
    gvk::Buffer mStorageBuffer;
    std::vector<gvk::ImageView> mImages;
    gvk::Sampler mSampler;
    gvk::DescriptorSet mDescriptorSet;
    std::vector<Sprite> mSprites;
};

} // namespace gfx
} // namespace dst
