
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

#include "shape-shooter/defines.hpp"

#include <vector>

namespace shape_shooter {

class ScoreBoard final
{
public:
    ScoreBoard() = default;
    ScoreBoard(ScoreBoard&& other) = default;
    ScoreBoard& operator=(ScoreBoard&& other) = default;
    static VkResult create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, ScoreBoard* pScoreBoard);
    ~ScoreBoard();
    void reset();

    int get_score();
    void add_points(int points);
    void increase_multiplier();
    void reset_score();
    void reset_multiplier();
    void update(float deltaTime);
    void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera) const;

private:
    static int load_high_score();
    static void save_high_score(int highScore);

    int mHighScore{ };
    int mScore{ };
    int mMultiplier{ };
    float mMultiplierTimer{ };
    std::shared_ptr<dst::text::Font> mspFont;
    dst::text::Mesh mScoreTextMesh;
    dst::text::Mesh mHighScoreTextMesh;
    dst::gfx::Renderer<dst::text::Font> mFontRenderer;

    ScoreBoard(const ScoreBoard&) = delete;
    ScoreBoard& operator=(const ScoreBoard&) = delete;
};

} // namespace shape_shooter
