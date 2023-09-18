
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

#include "dynamic-static.graphics/buffer.hpp"
#include "dynamic-static.graphics/defines.hpp"
#include "dynamic-static.graphics/renderer.hpp"
#include "dynamic-static/text.hpp"

namespace gvk {

template <>
inline auto get_vertex_description<dst::text::Mesh::Vertex>(uint32_t binding)
{
    return gvk::get_vertex_input_attribute_descriptions<
        glm::vec3,
        glm::vec2,
        glm::vec4
    >(binding);
}

} // namespace gvk

namespace dst {
namespace gfx {

template <>
class Renderer<dst::text::Font> final
{
public:
    static VkResult create(const gvk::Context& context, const gvk::RenderPass& renderPass, const dst::text::Font& font, Renderer<dst::text::Font>* pRenderer);
    const gvk::Pipeline& get_pipeline() const;
    const gvk::DescriptorSet& get_descriptor_set() const;
    void record_bind_cmds(const gvk::CommandBuffer& commandBuffer);

private:
    VkResult create_pipline(const gvk::RenderPass& renderPass, const dst::text::Font& font);
    VkResult create_image_views(const gvk::Context& context, const dst::text::Font& font);
    VkResult allocate_descriptor_set();
    void update_descriptor_set();

    gvk::Pipeline mPipeline;
    gvk::Sampler mSampler;
    std::vector<gvk::ImageView> mImageViews;
    gvk::DescriptorSet mDescriptorSet;
};

template <>
class Renderer<dst::text::Mesh> final
    : public dst::text::Mesh::Renderer
{
public:
    static VkResult create(const gvk::Device& device, const dst::text::Mesh& textMesh, const Renderer<dst::text::Font>& fontRenderer, Renderer<dst::text::Mesh>* pRenderer);
    void update(float deltaTime, const dst::text::Mesh& mesh);
    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const Renderer<dst::text::Font>& fontRenderer);
    gvk::math::Transform transform { };

private:
    VkResult create_uniform_buffer();
    VkResult allocate_descriptor_set(const Renderer<dst::text::Font>& fontRenderer);
    void update_descriptor_set();

    gvk::Device mDevice;
    uint32_t mIndexCount{ };
    VkDeviceSize mIndexDataOffset{ };
    gvk::Buffer mVertexIndexBuffer;
    gvk::Buffer mUniformBuffer;
    gvk::DescriptorSet mDescriptorSet;
};

} // namespace gfx
} // namespace dst
