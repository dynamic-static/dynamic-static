
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
#include "dynamic-static/text.hpp"

namespace dst {
namespace gfx {

template <typename T>
class Renderer final
{
};

template <>
class Renderer<dst::text::Font> final
{
public:
    static VkResult create(const gvk::DescriptorPool& descriptorPool, const dst::text::Font& font, Renderer<dst::text::Font>* pRenderer);

    void record_bind_cmds(const gvk::CommandBuffer& commandBuffer);

private:
    VkResult create_pipline(const gvk::Device& device, const dst::text::Font& font);
    VkResult create_image_views(const gvk::Device& device, const dst::text::Font& font);
    VkResult allocate_descriptor_set(const gvk::Device& device, const gvk::DescriptorPool& descriptorPool);
    void update_descriptor_set(const gvk::Device& device);

    gvk::Pipeline mPipeline;
    std::vector<gvk::ImageView> mImageViews;
    gvk::DescriptorSet mDescriptorSet;
};

template <>
class Renderer<dst::text::Mesh> final
{
public:
    static VkResult create(Renderer<dst::text::Mesh>* pRenderer);

    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer);

private:
    gvk::Buffer mBuffer;
};

} // namespace gfx
} // namespace dst
