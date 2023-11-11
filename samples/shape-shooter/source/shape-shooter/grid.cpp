
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

#include "shape-shooter/grid.hpp"

#include "shape-shooter/utilities.hpp"

namespace shape_shooter {

void Grid::PointMass::apply_force(const glm::vec3& force)
{
    acceleration += force * inverseMass;
}

void Grid::PointMass::increase_damping(float factor)
{
    damping *= factor;
}

void Grid::PointMass::update(float deltaTime)
{
    (void)deltaTime;
    velocity += acceleration;
    position += velocity; // *deltaTime;
    acceleration = { };
    if (glm::length2(velocity) < 0.001f * 0.001f) {
        velocity = { };
    }
    velocity *= damping;
    damping = 0.98f;
}

Grid::Spring::Spring(const std::vector<PointMass>& pointMasses, uint32_t p0_i, uint32_t p1_i, float stiffness, float damping)
    : mP0_i{ p0_i }
    , mP1_i{ p1_i }
    , mStiffness{ stiffness }
    , mDamping{ damping }
    , mTargetLength{ glm::distance(pointMasses[p0_i].position, pointMasses[p1_i].position) * 0.95f }
{
}

void Grid::Spring::update(std::vector<PointMass>& pointMasses)
{
    assert(mP0_i < pointMasses.size());
    assert(mP1_i < pointMasses.size());
    auto v = pointMasses[mP0_i].position - pointMasses[mP1_i].position;
    auto length = glm::length(v);
    if (mTargetLength < length) {
        v = (v / length) * (length - mTargetLength);
        auto dv = pointMasses[mP1_i].velocity - pointMasses[mP0_i].velocity;
        auto force = mStiffness * v - dv * mDamping;
        pointMasses[mP0_i].apply_force(-force);
        pointMasses[mP1_i].apply_force(force);
    }
}

VkResult Grid::create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, const CreateInfo* pCreateInfo, Grid* pGrid)
{
    assert(pCreateInfo);
    assert(pGrid);
    pGrid->mCreateInfo = *pCreateInfo;
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        auto lineRendererCreateInfo = gvk::get_default<dst::gfx::LineRenderer::CreateInfo>();
        gvk_result(dst::gfx::LineRenderer::create(gvkContext, renderPass, lineRendererCreateInfo, &pGrid->mLineRenderer));
        pGrid->create_grid();
    } gvk_result_scope_end;
    return gvkResult;
}

void Grid::apply_directed_force(const glm::vec3& force, const glm::vec3& position, float radius)
{
    (void)force;
    (void)position;
    (void)radius;
#if 0
    for (auto& pointMass : mPointMasses) {
        if (glm::distance2(position, pointMass.position) < radius * radius) {
            pointMass.apply_force(10.0f * force / (10.0f + glm::distance(position, pointMass.position)));
        }
    }
#endif
}

void Grid::apply_implosive_force(float force, const glm::vec3& position, float radius)
{
    (void)force;
    (void)position;
    (void)radius;
#if 0
    for (auto& pointMass : mPointMasses) {
        auto distanceSqrd = glm::distance2(position, pointMass.position);
        if (distanceSqrd < radius * radius) {
            pointMass.apply_force(10.0f * force * (position - pointMass.position) / (100.0f + distanceSqrd));
            pointMass.increase_damping(0.6f);
        }
    }
#endif
}

void Grid::apply_explosive_force(float force, const glm::vec3& position, float radius)
{
    (void)force;
    (void)position;
    (void)radius;
#if 0
    for (auto& pointMass : mPointMasses) {
        auto distanceSqrd = glm::distance2(position, pointMass.position);
        if (distanceSqrd < radius * radius) {
            // pointMass.apply_force(100.0f * force * (pointMass.position - position) / (10000.0f + distanceSqrd));
            pointMass.apply_force(100.0f * force * (pointMass.position - position) / (10000.0f + distanceSqrd));
            pointMass.increase_damping(0.6f);
        }
    }
#endif
}

void Grid::update(float deltaTime)
{
    auto index = [&](uint32_t x, uint32_t y) { return y * (mCreateInfo.cells.x + 1) + x; };
    auto point = [](const PointMass& p, float w)
    {
        return dst::gfx::Point{
            glm::vec4 { p.position.x, p.position.y, p.position.z, 1 },
            glm::vec4 { 0.117647059f, 0.117647059f, 0.545098066f, 1 }, // 0.333333343f
            glm::vec4 { w, 1, 0, 0 },
        };
    };
    for (auto& spring : mSprings) {
        spring.update(mPointMasses);
    }
    for (uint32_t y = 0; y < mCreateInfo.cells.y; ++y) {
        for (uint32_t x = 0; x < mCreateInfo.cells.x; ++x) {
            mPointMasses[index(x, y)].update(deltaTime);
        }
    }
    mPoints.clear();
    for (uint32_t y = 0; y < mCreateInfo.cells.y; ++y) {
        for (uint32_t x = 0; x < mCreateInfo.cells.x; ++x) {
            if (!y) {
                mPoints.push_back(point(mPointMasses[index(x, y)], 3));
            }
            mPoints.push_back(point(mPointMasses[index(x + 1, y    )], 3));
            mPoints.push_back(point(mPointMasses[index(x + 1, y + 1)], 3));
            mPoints.push_back(point(mPointMasses[index(x,     y + 1)], 3));
            if (!x) {
                mPoints.push_back(point(mPointMasses[index(x, y)], 3));
            }
            mPoints.push_back(mPoints.back());
            mPoints.back().width.g = 0;
        }
    }
    mLineRenderer.submit((uint32_t)mPoints.size(), mPoints.data());
}

void Grid::record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera, const glm::vec2& resolution) const
{
    mLineRenderer.record_draw_cmds(commandBuffer, camera, resolution);
}

void Grid::create_grid()
{
    auto w = mCreateInfo.cells.x + 1;
    auto h = mCreateInfo.cells.y + 1;
    auto halfExtent = mCreateInfo.extent * 0.5f;
    auto cellExtent = mCreateInfo.extent / glm::vec2{ (float)mCreateInfo.cells.x, (float)mCreateInfo.cells.y };

    // TODO : Documentation
    mPointMasses.reserve(w * h /* grid */ + w * 2 + h * 2 /* border */ + w * h / 9 /* anchors */);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            PointMass pointMass{ };
            pointMass.position = { -halfExtent.x + cellExtent.x * x, -0.1f, -halfExtent.y + cellExtent.y * y };
            pointMass.inverseMass = 1;
            mPointMasses.push_back(pointMass);
        }
    }

    // TODO : Documentation
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            auto i_0 = y * w + x;
        
            // TODO : Documentation
            if (!x || !y || x == w - 1 || y == h - 1) { // Anchor the border of the grid
                // TODO : Fix missing edge anchors...
                mPointMasses.push_back(mPointMasses[i_0]);
                mPointMasses.back().inverseMass = 0;
                mSprings.push_back(Spring(mPointMasses, (uint32_t)mPointMasses.size() - 1, i_0, 0.1f, 0.1f));
            } else if (!(x % 3) && !(y % 3)) { // Loosely anchor 1/9th of the point masses
                mPointMasses.push_back(mPointMasses[i_0]);
                mPointMasses.back().inverseMass = 0;
                mSprings.push_back(Spring(mPointMasses, (uint32_t)mPointMasses.size() - 1, i_0, 0.002f, 0.02f));
            }
        
            // TODO : Documentation
            const float Stiffness = 0.28f;
            const float Damping = 0.06f;
            if (x) {
                auto i_1 = y * w + (x - 1);
                mSprings.push_back(Spring(mPointMasses, i_0, i_1, Stiffness, Damping));
            }
            if (y) {
                auto i_1 = (y - 1) * w + x;
                mSprings.push_back(Spring(mPointMasses, i_0, i_1, Stiffness, Damping));
            }
        }
    }
}

} // namespace shape_shooter
