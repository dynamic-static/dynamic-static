
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

#include "dynamic-static.graphics/line-renderer.hpp"

namespace shape_shooter {

class Grid final
{
public:
    struct CreateInfo
    {
        glm::vec2 extent{ };
        glm::uvec2 cells{ };
    };

    struct PointMass
    {
        void apply_force(const glm::vec3& force);
        void increase_damping(float factor);
        void update();

        glm::vec3 position{ };
        glm::vec3 velocity{ };
        glm::vec3 acceleration{ };
        float inverseMass{ };
        float damping{ 0.98f };
    };

    class Spring final
    {
    public:
        Spring(const std::vector<PointMass>& pointMasses, uint32_t p0_i, uint32_t p1_i, float stiffness, float damping);
        void update(std::vector<PointMass>& pointMasses);

    private:
        uint32_t mP0_i{ };
        uint32_t mP1_i{ };
        float mTargetLength{ };
        float mStiffness{ };
        float mDamping{ };
    };

    static VkResult create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, const CreateInfo* pCreateInfo, Grid* pGrid);
    void apply_directed_force(const glm::vec3& force, const glm::vec3& position, float radius);
    void apply_implosive_force(float force, const glm::vec3& position, float radius);
    void apply_explosive_force(float force, const glm::vec3& position, float radius);
    void update();
    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera, const glm::vec2& resolution) const;

private:
    void create_grid();

    CreateInfo mCreateInfo{ };
    std::vector<Spring> mSprings;
    std::vector<PointMass> mPointMasses;
    dst::gfx::LineRenderer mLineRenderer;
    std::vector<dst::gfx::Point> mPoints;
};

} // namespace shape_shooter
