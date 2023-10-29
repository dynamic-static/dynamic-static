
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

#include "dynamic-static.graphics/coordinate-renderer.hpp"

namespace dst {
namespace gfx {

VkResult CoordinateRenderer::create(const gvk::Context& gvkContext, const CreateInfo& createInfo, CoordinateRenderer* pCoordinateRenderer)
{
    assert(createInfo.renderPass);
    assert(createInfo.pTtfFilePath);
    assert(createInfo.fontSize);
    assert(pCoordinateRenderer);
    pCoordinateRenderer->reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        LineRenderer::CreateInfo lineRendererCreateInfo{ };
        gvk_result(LineRenderer::create(gvkContext, createInfo.renderPass, lineRendererCreateInfo, &pCoordinateRenderer->mLineRenderer));
        text::Font::create(createInfo.pTtfFilePath, nullptr, createInfo.fontSize, &pCoordinateRenderer->mspFont);
        gvk_result(Renderer<text::Font>::create(gvkContext, createInfo.renderPass, *pCoordinateRenderer->mspFont, &pCoordinateRenderer->mFontRenderer));
        pCoordinateRenderer->set_min(createInfo.min);
        pCoordinateRenderer->set_max(createInfo.max);
        pCoordinateRenderer->set_extent(createInfo.extent);
        pCoordinateRenderer->set_origin(createInfo.origin);
        pCoordinateRenderer->set_major_tick(createInfo.majorTick);
        pCoordinateRenderer->set_minor_tick(createInfo.minorTick);
    } gvk_result_scope_end;
    return gvkResult;
}

CoordinateRenderer::~CoordinateRenderer()
{
    reset();
}

void CoordinateRenderer::reset()
{
    mLineRenderer.reset();
    mspFont.reset();
    mFontRenderer = { };
    mTextMeshes.clear();
}

void CoordinateRenderer::update()
{
    if (mUpdate) {
        mUpdate = false;
        mPoints.clear();
        std::vector<dst::text::Mesh> textMeshes;
        auto getTextMesh = [&](auto& textMesh)
        {
            if (!mTextMeshes.empty()) {
                textMesh = std::move(mTextMeshes.back());
                textMesh.set_text(std::string{ });
                mTextMeshes.pop_back();
            } else {
                textMesh.create_renderer<Renderer<dst::text::Mesh>>(
                    [&](const auto& /* textMesh */, auto& renderer)
                    {
                        return Renderer<text::Mesh>::create(mFontRenderer.get_pipeline().get<gvk::Device>(), textMesh, mFontRenderer, &renderer);
                    }
                );
            }
        };
        ///////////////////////////////////////////////////////////////////////////////

        Point point{ };
        point.position.x = mMin.x;
        point.color = gvk::math::Color::Red;
        point.width.r = 2;
        mPoints.push_back(point);
        point.position.x = mMax.x;
        point.color = gvk::math::Color::Red;
        point.width.r = 2;
        mPoints.push_back(point);
        mPoints.push_back(mPoints.back());
        mPoints.back().width.g = 0;

        point = { };
        point.position.y = mMin.y;
        point.color = gvk::math::Color::Green;
        point.width.r = 2;
        mPoints.push_back(point);
        point.position.y = mMax.y;
        point.color = gvk::math::Color::Green;
        point.width.r = 2;
        mPoints.push_back(point);
        mPoints.push_back(mPoints.back());
        mPoints.back().width.g = 0;

        point = { };
        point.position.z = mMin.z;
        point.color = gvk::math::Color::Blue;
        point.width.r = 2;
        mPoints.push_back(point);
        point.position.z = mMax.z;
        point.color = gvk::math::Color::Blue;
        point.width.r = 2;
        mPoints.push_back(point);
        mPoints.push_back(mPoints.back());
        mPoints.back().width.g = 0;

        ///////////////////////////////////////////////////////////////////////////////
        float xMajor = mMin.x;
        uint32_t xMajorTickCount = (uint32_t)((mMax.x - mMin.x) / mMajorTick.x);
        for (uint32_t i = 0; i <= xMajorTickCount; ++i) {
            mPoints.emplace_back();
            mPoints.back().position.x = xMajor;
            mPoints.back().color = gvk::math::Color::Red;
            mPoints.back().width.r = 2;
            mPoints.push_back(mPoints.back());
            mPoints.back().position.y = 1;
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;

            dst::text::Mesh textMesh;
            getTextMesh(textMesh);
            textMesh.set_font(mspFont);
            textMesh.set_text(std::to_string(xMajor));
            assert(textMesh.get_renderers().size() == 1);
            auto pTextMeshRenderer = (dst::gfx::Renderer<dst::text::Mesh>*)textMesh.get_renderers()[0].get();
            pTextMeshRenderer->transform = { };
            pTextMeshRenderer->transform.translation = { xMajor, 1, 0 };
            pTextMeshRenderer->transform.scale *= 0.025f;
            textMeshes.push_back(std::move(textMesh));

            xMajor += mMajorTick.x;
        }
        float xMinor = mMin.x;
        uint32_t xMinorTickCount = (uint32_t)((mMax.x - mMin.x) / mMinorTick.x);
        for (uint32_t i = 0; i <= xMinorTickCount; ++i) {
            mPoints.emplace_back();
            mPoints.back().position.x = xMinor;
            mPoints.back().color = gvk::math::Color::Red;
            mPoints.back().width.r = 2;
            mPoints.push_back(mPoints.back());
            mPoints.back().position.y = 0.5f;
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;
            xMinor += mMinorTick.x;
        }
        ///////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////
        float yMajor = mMin.y;
        uint32_t yMajorTickCount = (uint32_t)((mMax.y - mMin.y) / mMajorTick.y);
        for (uint32_t i = 0; i <= yMajorTickCount; ++i) {
            mPoints.emplace_back();
            mPoints.back().position.y = yMajor;
            mPoints.back().color = gvk::math::Color::Green;
            mPoints.back().width.r = 2;
            mPoints.push_back(mPoints.back());
            mPoints.back().position.z = 1;
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;

            dst::text::Mesh textMesh;
            getTextMesh(textMesh);
            textMesh.set_font(mspFont);
            textMesh.set_text(std::to_string(yMajor));
            assert(textMesh.get_renderers().size() == 1);
            auto pTextMeshRenderer = (dst::gfx::Renderer<dst::text::Mesh>*)textMesh.get_renderers()[0].get();
            pTextMeshRenderer->transform = { };
            pTextMeshRenderer->transform.translation = { 0, yMajor, 1 };
            pTextMeshRenderer->transform.scale *= 0.025f;
            textMeshes.push_back(std::move(textMesh));

            yMajor += mMajorTick.y;
        }
        float yMinor = mMin.y;
        uint32_t yMinorTickCount = (uint32_t)((mMax.y - mMin.y) / mMinorTick.y);
        for (uint32_t i = 0; i <= yMinorTickCount; ++i) {
            mPoints.emplace_back();
            mPoints.back().position.y = yMinor;
            mPoints.back().color = gvk::math::Color::Green;
            mPoints.back().width.r = 2;
            mPoints.push_back(mPoints.back());
            mPoints.back().position.z = 0.5f;
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;
            yMinor += mMinorTick.y;
        }
        ///////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////
        float zMajor = mMin.z;
        uint32_t zMajorTickCount = (uint32_t)((mMax.z - mMin.z) / mMajorTick.z);
        for (uint32_t i = 0; i <= zMajorTickCount; ++i) {
            mPoints.emplace_back();
            mPoints.back().position.z = zMajor;
            mPoints.back().color = gvk::math::Color::Blue;
            mPoints.back().width.r = 2;
            mPoints.push_back(mPoints.back());
            mPoints.back().position.x = 1;
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;

            dst::text::Mesh textMesh;
            getTextMesh(textMesh);
            textMesh.set_font(mspFont);
            textMesh.set_text(std::to_string(zMajor));
            assert(textMesh.get_renderers().size() == 1);
            auto pTextMeshRenderer = (dst::gfx::Renderer<dst::text::Mesh>*)textMesh.get_renderers()[0].get();
            pTextMeshRenderer->transform = { };
            pTextMeshRenderer->transform.translation = { 1, 0, zMajor };
            pTextMeshRenderer->transform.scale *= 0.025f;
            textMeshes.push_back(std::move(textMesh));

            zMajor += mMajorTick.z;
        }
        float zMinor = mMin.z;
        uint32_t zMinorTickCount = (uint32_t)((mMax.z - mMin.z) / mMinorTick.z);
        for (uint32_t i = 0; i <= zMinorTickCount; ++i) {
            mPoints.emplace_back();
            mPoints.back().position.z = zMinor;
            mPoints.back().color = gvk::math::Color::Blue;
            mPoints.back().width.r = 2;
            mPoints.push_back(mPoints.back());
            mPoints.back().position.x = 0.5f;
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;
            zMinor += mMinorTick.z;
        }
        ///////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////
        mLineRenderer.submit((uint32_t)mPoints.size(), mPoints.data());
        std::swap(textMeshes, mTextMeshes);
        for (auto& textMesh : mTextMeshes) {
            textMesh.update(0);
        }
    }
}

void CoordinateRenderer::record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera, const glm::vec2& resolution) const
{
    for (const auto& textMesh : mTextMeshes) {
        const auto& fontPipeline = mFontRenderer.get_pipeline();
        const auto& fontDescriptorSet = mFontRenderer.get_descriptor_set();
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, fontPipeline);
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, fontPipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)fontDescriptorSet, 0, nullptr);
        // TODO : This is kludgy...
        assert(textMesh.get_renderers().size() == 1);
        auto pTextMeshRenderer = (dst::gfx::Renderer<dst::text::Mesh>*)textMesh.get_renderers()[0].get();
        pTextMeshRenderer->record_draw_cmds(commandBuffer, mFontRenderer);
    }
    mLineRenderer.record_draw_cmds(commandBuffer, camera, resolution);
}

const glm::vec3& CoordinateRenderer::get_min() const
{
    return mMin;
}

void CoordinateRenderer::set_min(const glm::vec3& min)
{
    set_member_value(min, mMin);
}

const glm::vec3& CoordinateRenderer::get_max() const
{
    return mMax;
}

void CoordinateRenderer::set_max(const glm::vec3& max)
{
    set_member_value(max, mMax);
}

const glm::vec3& CoordinateRenderer::get_extent() const
{
    return mExtent;
}

void CoordinateRenderer::set_extent(const glm::vec3& extent)
{
    set_member_value(extent, mExtent);
}

const glm::vec3& CoordinateRenderer::get_origin() const
{
    return mOrigin;
}

void CoordinateRenderer::set_origin(const glm::vec3& origin)
{
    set_member_value(origin, mOrigin);
}

const glm::vec3& CoordinateRenderer::get_major_tick() const
{
    return mMajorTick;
}

void CoordinateRenderer::set_major_tick(const glm::vec3& majorTick)
{
    set_member_value(majorTick, mMajorTick);
}

const glm::vec3& CoordinateRenderer::get_minor_tick() const
{
    return mMinorTick;
}

void CoordinateRenderer::set_minor_tick(const glm::vec3& minorTick)
{
    set_member_value(minorTick, mMinorTick);
}

template <typename T>
void CoordinateRenderer::set_member_value(const T& value, T& member)
{
    if (member != value) {
        member = value;
        mUpdate = true;
    }
}

} // namespace gfx
} // namespace dst
