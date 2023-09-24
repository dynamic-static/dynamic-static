
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
#include "dynamic-static.graphics/renderer.hpp"

#include "gvk-math.hpp"

#include <set>
#include <unordered_map>

namespace dst {
namespace gfx {

struct Sprite
{
    glm::vec4 color { gvk::math::Color::White };
    gvk::math::Transform transform { };
    uint64_t textureId { 0 };
};

template <>
class Renderer<Sprite> final
{
public:
    struct CreateInfo
    {
        uint32_t filePathCount { };
        const char* const* ppFilePaths { };
    };

    static VkResult create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, const CreateInfo& createInfo, Renderer<Sprite>* pRenderer);
    ~Renderer();
    void reset();
    void begin_sprite_batch();
    void submit(const Sprite& sprite);
    void end_sprite_batch();
    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera) const;

private:
    VkResult create_pipeline(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass);
    VkResult create_image_views(const gvk::Context& gvkContext, const CreateInfo& createInfo);
    VkResult allocate_descriptor_set(const gvk::Context& gvkContext);

    struct GlSprite
    {
        glm::vec4 extent { };
        glm::vec2 uvMin { };
        glm::vec2 uvMax { };
        glm::vec4 color { };
        glm::mat4 model { };
    };

    gvk::Pipeline mPipeline;
    gvk::Buffer mStorageBuffer;
    std::unordered_map<std::string, gvk::ImageView> mImages;
    gvk::DescriptorSet mDescriptorSet;
    std::vector<GlSprite> mSprites;
};

} // namespace gfx
} // namespace dst
