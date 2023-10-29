
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
#include "dynamic-static.graphics/text.hpp"
#include "dynamic-static.graphics/line-renderer.hpp"
#include "dynamic-static/text.hpp"

#include <memory>

namespace dst {
namespace gfx {

class CoordinateRenderer final
{
public:
    class CreateInfo final
    {
    public:
        gvk::RenderPass renderPass;
        glm::vec3 min      { -32, -32, -32 };
        glm::vec3 max      {  32,  32,  32 };
        glm::vec3 extent   {  32,  32,  32 };
        glm::vec3 origin   {   0,   0,   0 };
        glm::vec3 majorTick{   4,   4,   4 };
        glm::vec3 minorTick{   1,   1,   1 };
        const char* pTtfFilePath{ };
        float fontSize{ 32 };
    };

    CoordinateRenderer() = default;
    static VkResult create(const gvk::Context& gvkContext, const CreateInfo& createInfo, CoordinateRenderer* pCoordinateRenderer);
    ~CoordinateRenderer();
    void reset();

    const glm::vec3& get_min() const;
    void set_min(const glm::vec3& min);
    const glm::vec3& get_max() const;
    void set_max(const glm::vec3& max);
    const glm::vec3& get_extent() const;
    void set_extent(const glm::vec3& extent);
    const glm::vec3& get_origin() const;
    void set_origin(const glm::vec3& origin);
    const glm::vec3& get_major_tick() const;
    void set_major_tick(const glm::vec3& majorTick);
    const glm::vec3& get_minor_tick() const;
    void set_minor_tick(const glm::vec3& minorTick);

    void update();
    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera, const glm::vec2& resolution) const;

private:
    template <typename T>
    void set_member_value(const T& value, T& member);

    bool mUpdate{ };
    glm::vec3 mMin{ };
    glm::vec3 mMax{ };
    glm::vec3 mExtent{ };
    glm::vec3 mOrigin{ };
    glm::vec3 mMajorTick{ };
    glm::vec3 mMinorTick{ };
    LineRenderer mLineRenderer;
    std::vector<dst::gfx::Point> mPoints;
    std::shared_ptr<dst::text::Font> mspFont;
    dst::gfx::Renderer<dst::text::Font> mFontRenderer;
    std::vector<dst::text::Mesh> mTextMeshes;

    CoordinateRenderer(const CoordinateRenderer&) = delete;
    CoordinateRenderer& operator=(const CoordinateRenderer&) = delete;
};

} // namespace gfx
} // namespace dst
