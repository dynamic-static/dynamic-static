
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

#include "shape-shooter/score-board.hpp"
#include "shape-shooter/context.hpp"

#include <filesystem>
#include <fstream>

namespace shape_shooter {

static constexpr int MaxMultiplier{ 20 };
static constexpr float MultiplierExpiryTime{ 0.8f };
static const std::filesystem::path HighScoreFilePath { "highscore.txt" };

static dst::gfx::Renderer<dst::text::Mesh>* get_text_mesh_renderer(const dst::text::Mesh& textMesh)
{
    assert(textMesh.get_renderers().size() == 1);
    assert(textMesh.get_renderers().back());
    return (dst::gfx::Renderer<dst::text::Mesh>*)textMesh.get_renderers()[0].get();
}

VkResult ScoreBoard::create(const gvk::Context& gvkContext, const gvk::RenderPass& renderPass, ScoreBoard* pScoreBoard)
{
    assert(pScoreBoard);
    pScoreBoard->reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        dst::text::Font::create("C:\\Windows\\Fonts\\bauhs93.ttf", nullptr, 64, &pScoreBoard->mspFont);
        dst::gfx::Renderer<dst::text::Font>::create(gvkContext, renderPass, *pScoreBoard->mspFont, &pScoreBoard->mFontRenderer);
        pScoreBoard->mScoreTextMesh.set_font(pScoreBoard->mspFont);
        pScoreBoard->mHighScoreTextMesh.set_font(pScoreBoard->mspFont);
        pScoreBoard->mLivesTextMesh.set_font(pScoreBoard->mspFont);
        auto createTextMeshRenderer = [&](const auto& textMesh, auto& renderer)
        {
            return dst::gfx::Renderer<dst::text::Mesh>::create(gvkContext.get_devices()[0], textMesh, pScoreBoard->mFontRenderer, &renderer);
        };
        gvk_result(pScoreBoard->mScoreTextMesh.create_renderer<dst::gfx::Renderer<dst::text::Mesh>>(createTextMeshRenderer));
        gvk_result(pScoreBoard->mHighScoreTextMesh.create_renderer<dst::gfx::Renderer<dst::text::Mesh>>(createTextMeshRenderer));
        gvk_result(pScoreBoard->mLivesTextMesh.create_renderer<dst::gfx::Renderer<dst::text::Mesh>>(createTextMeshRenderer));
        pScoreBoard->reset_score();
    } gvk_result_scope_end;
    return gvkResult;
}

ScoreBoard::~ScoreBoard()
{
    reset();
}

void ScoreBoard::reset()
{
    save_high_score(mHighScore);
    mHighScore = 0;
    mScore = 0;
    mMultiplier = 0;
    mMultiplierTimer = 0;
    mspFont.reset();
    mScoreTextMesh = { };
    mHighScoreTextMesh = { };
    mLivesTextMesh = { };
    mFontRenderer = { };
}

const dst::gfx::Renderer<dst::text::Font>& ScoreBoard::get_font_renderer() const
{
    return mFontRenderer;
}

int ScoreBoard::get_score()
{
    return mScore;
}

void ScoreBoard::add_points(int points)
{
    mScore += points * mMultiplier;
    if (mHighScore <= mScore) {
        mHighScore = mScore;
    }
}

void ScoreBoard::increase_multiplier()
{
    mMultiplierTimer = MultiplierExpiryTime;
    mMultiplier = std::clamp(mMultiplier + 1, 1, MaxMultiplier);
}

void ScoreBoard::reset_score()
{
    save_high_score(mHighScore);
    mHighScore = load_high_score();
    reset_multiplier();
    mScore = 0;
}

void ScoreBoard::reset_multiplier()
{
    mMultiplier = 1;
}

void ScoreBoard::update()
{
    auto deltaTime = Context::instance().clock.elapsed<gvk::system::Seconds<float>>();

    mScoreTextMesh.set_text(std::to_string(mScore));
    auto pTextMeshRenderer = get_text_mesh_renderer(mScoreTextMesh);
    pTextMeshRenderer->transform.rotation = glm::angleAxis(glm::radians(180.0f), glm::vec3{ 0, 1, 0 });
    pTextMeshRenderer->transform.translation.y = 64;
    mScoreTextMesh.update(deltaTime);

    mHighScoreTextMesh.set_text("hi : " + std::to_string(mHighScore));
    pTextMeshRenderer = get_text_mesh_renderer(mHighScoreTextMesh);
    pTextMeshRenderer->transform.rotation = glm::angleAxis(glm::radians(180.0f), glm::vec3{ 0, 1, 0 });
    pTextMeshRenderer->transform.translation.y = 32;
    pTextMeshRenderer->transform.scale = { 0.5f, 0.5f, 0.5f };
    mHighScoreTextMesh.update(deltaTime);

    mLivesTextMesh.set_text("lives : " + std::to_string(3));
    pTextMeshRenderer = get_text_mesh_renderer(mLivesTextMesh);
    pTextMeshRenderer->transform.rotation = glm::angleAxis(glm::radians(180.0f), glm::vec3{ 0, 1, 0 });
    pTextMeshRenderer->transform.translation.y = 8;
    pTextMeshRenderer->transform.scale = { 0.5f, 0.5f, 0.5f };
    mLivesTextMesh.update(deltaTime);
}

void ScoreBoard::record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::math::Camera& camera) const
{
    (void)camera;
    auto bindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    const auto& fontPipeline = mFontRenderer.get_pipeline();
    const auto& fontPipelineLayout = fontPipeline.get<gvk::PipelineLayout>();
    const auto& fontDescriptorSet = mFontRenderer.get_descriptor_set().get<VkDescriptorSet>();
    const auto& dispatchTable = commandBuffer.get<gvk::Device>().get<gvk::DispatchTable>();
    dispatchTable.gvkCmdBindPipeline(commandBuffer, bindPoint, fontPipeline);
    dispatchTable.gvkCmdBindDescriptorSets(commandBuffer, bindPoint, fontPipelineLayout, 1, 1, &fontDescriptorSet, 0, nullptr);
    get_text_mesh_renderer(mScoreTextMesh)->record_draw_cmds(commandBuffer, mFontRenderer);
    get_text_mesh_renderer(mHighScoreTextMesh)->record_draw_cmds(commandBuffer, mFontRenderer);
    get_text_mesh_renderer(mLivesTextMesh)->record_draw_cmds(commandBuffer, mFontRenderer);
}

int ScoreBoard::load_high_score()
{
    int highScore = 0;
    std::ifstream highScoreFile(HighScoreFilePath);
    if (highScoreFile.is_open()) {
        highScoreFile >> highScore;
    }
    return highScore;
}

void ScoreBoard::save_high_score(int highScore)
{
    if (load_high_score() < highScore) {
        std::ofstream highScoreFile(HighScoreFilePath);
        if (highScoreFile.is_open()) {
            highScoreFile << highScore;
        }
    }
}

} // namespace shape_shooter
