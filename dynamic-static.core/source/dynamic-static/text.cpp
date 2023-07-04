
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

#include "dynamic-static/text.hpp"

#include <iostream>

namespace dst {
namespace text {

bool is_printable(int32_t codepoint)
{
    if (Glyph::NullId <= codepoint && codepoint <= Glyph::SpaceId) { // ASCII control characters
        return false;
    }
    if (127 <= codepoint && codepoint <= 159) { // C1 extended control characters
        return false;
    }
    if (codepoint == Glyph::LeftToRightId ||
        codepoint == Glyph::RightToLeftId ||
        codepoint == Glyph::LineSeperatorId ||
        codepoint == Glyph::ParagraphSeperatorId) {
        return false;
    }
    return true;
}

bool operator==(const Glyph& lhs, const Glyph& rhs)
{
    return lhs.codepoint == rhs.codepoint;
}

bool operator!=(const Glyph& lhs, const Glyph& rhs)
{
    return lhs.codepoint != rhs.codepoint;
}

bool operator<(const Glyph& lhs, const Glyph& rhs)
{
    return lhs.codepoint < rhs.codepoint;
}

bool operator>(const Glyph& lhs, const Glyph& rhs)
{
    return lhs.codepoint > rhs.codepoint;
}

bool operator<=(const Glyph& lhs, const Glyph& rhs)
{
    return lhs.codepoint <= rhs.codepoint;
}

bool operator>=(const Glyph& lhs, const Glyph& rhs)
{
    return lhs.codepoint >= rhs.codepoint;
}

void Font::create(const char* pFilePath, const char* pCharacterSet, float size, std::shared_ptr<Font>* pspFont)
{
    assert(pFilePath);
    assert(pspFont);
    if (!pCharacterSet) {
        pCharacterSet = DefaultCharacterSet;
    }
    *pspFont = std::make_shared<Font>();
    auto& font = **pspFont;
    stbtt_fontinfo fontInfo { };
    auto ttfBytes = read_bytes(pFilePath);
    if (stbtt_InitFont(&fontInfo, ttfBytes.data(), 0)) {
        // auto scale = stbtt_ScaleForPixelHeight(&fontInfo, pFont->mLineHeight);
        auto scale = stbtt_ScaleForPixelHeight(&fontInfo, size);
        int ascent, descent, lineGap;
        stbtt_GetFontVMetrics(&fontInfo, &ascent, &descent, &lineGap);
        font.mAscent = ascent * scale;
        font.mDescent = descent * scale;
        font.mLineHeight = lineGap * scale;

        // TODO : Documentation
        std::set<char> characters;
        std::vector<BinPackEntry<char>> binPackEntries;
        auto pChar = pCharacterSet;
        while (*pChar) {
            if (characters.insert(*pChar).second) {
                // TODO : Remove control characters
                // TODO : Remove characters not present in font
                int ix0, iy0, ix1, iy1;
                stbtt_GetCodepointBitmapBox(&fontInfo, (int)*pChar, scale, scale, &ix0, &iy0, &ix1, &iy1);
                BinPackEntry<char> binPackEntry { };
                binPackEntry.cell.width = ix1 - ix0;
                binPackEntry.cell.height = iy1 - iy0;
                binPackEntry.value = *pChar;
                binPackEntries.push_back(binPackEntry);
            }
            ++pChar;
        }

        // TODO : Documentation
        BinPackInfo binPackInfo { };
        bin_pack(&binPackInfo, binPackEntries.size(), binPackEntries.data());

        // TODO : Documentation
        font.mAtlas.width = binPackInfo.width;
        font.mAtlas.height = binPackInfo.height;
        font.mAtlas.pages.resize(binPackInfo.pageCount, Image<R8G8B8A8Unorm>({ font.mAtlas.width, font.mAtlas.height }));

        // TODO : Documentation
        for (const auto& binPackEntry : binPackEntries) {

            std::cout << binPackEntry.value << " : " << binPackEntry.cell.width << " : " << binPackEntry.cell.height << " : " << binPackEntry.page << std::endl;

            // TODO : Documentation
            assert(binPackEntry.page < font.mAtlas.pages.size());
            auto& page = font.mAtlas.pages[binPackEntry.page];
            const auto& cell = binPackEntry.cell;
            auto u = (uint32_t)cell.x + 2;
            auto v = (uint32_t)cell.y + 2;
            auto pData = (unsigned char*)&page[{ u, v }];
            auto stride = page.get_extent()[0];
            auto codepoint = (int32_t)binPackEntry.value;
            stbtt_MakeCodepointBitmap(&fontInfo, pData, cell.width, cell.height, stride, scale, scale, codepoint);

            // TODO : Documentation
            int advanceWidth, leftSideBearing;
            stbtt_GetCodepointHMetrics(&fontInfo, codepoint, &advanceWidth, &leftSideBearing);
            int ix0, iy0, ix1, iy1;
            stbtt_GetCodepointBitmapBox(&fontInfo, (int)*pChar, scale, scale, &ix0, &iy0, &ix1, &iy1);

            // TODO : Documentation
            Glyph glyph { };
            glyph.codepoint = codepoint;
            glyph.page = binPackEntry.page;
            glyph.texcoord.x = (cell.x + 2 + cell.width * 0.5f) / (float)page.get_extent()[0];
            glyph.texcoord.y = (cell.y + 2 + cell.height * 0.5f) / (float)page.get_extent()[1];
            glyph.extent = { (float)cell.width, (float)cell.height };
            glyph.offset = { (float)ix0 * scale, (float)iy0 * scale };
            glyph.xAdvance = (float)advanceWidth * scale;
            font.mGlyphs.insert({ glyph.codepoint, glyph });
        }

        // TODO : Documentation
        for (auto ch0 : characters) {
            for (auto ch1 : characters) {
                auto kerning = stbtt_GetCodepointKernAdvance(&fontInfo, ch0, ch1);
                if (kerning) {
                    // font.mKerningPairs[{ ch0, ch1 }] = kerning * scale;
                }
            }
        }
    }
}

float Font::get_ascent() const
{
    return mAscent;
}

float Font::get_descent() const
{
    return mDescent;
}

float Font::get_line_height() const
{
    return mLineHeight;
}

float Font::get_base_line() const
{
    return mBaseLine;
}

const Atlas& Font::get_atlas() const
{
    return mAtlas;
}

const Glyph& Font::get_glyph(int32_t codepoint) const
{
    static const Glyph NullGlyph;
    auto itr = mGlyphs.find(codepoint);
    return itr != mGlyphs.end() ? itr->second : NullGlyph;
}

float Font::get_kerning(int32_t lhsCodepoint, int32_t rhsCodepoint) const
{
    auto itr = mKerningPairs.find({ lhsCodepoint, rhsCodepoint });
    return itr != mKerningPairs.end() ? itr->second : 0;
}

Mesh::Controller::~Controller()
{
}

Mesh::Renderer::~Renderer()
{
}

void Mesh::Controller::update(float deltaTime, const Mesh& mesh, size_t i, std::array<Vertex, 4>& vertices)
{
    (void)deltaTime;
    (void)mesh;
    (void)i;
    (void)vertices;
}

void Mesh::Controller::update(float deltaTime, const Mesh& mesh, std::vector<Vertex>& vertices)
{
    (void)deltaTime;
    (void)mesh;
    (void)vertices;
}

void Mesh::Renderer::update(float deltaTime, const Mesh& mesh)
{
    (void)deltaTime;
    (void)mesh;
}

const std::string& Mesh::get_text() const
{
    return mText;
}

void Mesh::set_text(const std::string& text)
{
    set_member_value(text, mText);
}

const std::shared_ptr<Font>& Mesh::get_font() const
{
    return mspFont;
}

void Mesh::set_font(const std::shared_ptr<Font>& spFont)
{
    set_member_value(spFont, mspFont);
}

const std::vector<Mesh::Vertex>& Mesh::get_vertices() const
{
    return mVertices;
}

const std::vector<uint16_t>& Mesh::get_indices() const
{
    return mIndices;
}

float Mesh::get_glyph_spacing() const
{
    return mGlyphSpacing;
}

void Mesh::set_glpyh_spacing(float glyphSpacing)
{
    set_member_value(glyphSpacing, mGlyphSpacing);
}

float Mesh::get_line_spacing() const
{
    return mLineSpacing;
}

void Mesh::set_line_spacing(float lineSpacing)
{
    set_member_value(lineSpacing, mLineSpacing);
}

bool Mesh::kerning_enabled() const
{
    return mKerningEnabled;
}

void Mesh::kerning_enabled(bool kerningEnabled)
{
    set_member_value(kerningEnabled, mKerningEnabled);
}

const glm::vec4& Mesh::get_color() const
{
    return mColor;
}

void Mesh::set_color(const glm::vec4& color)
{
    set_member_value(color, mColor);
}

const std::vector<std::unique_ptr<Mesh::Controller>>& Mesh::get_controllers() const
{
    return mControllers;
}

const std::vector<std::unique_ptr<Mesh::Renderer>>& Mesh::get_renderers() const
{
    return mRenderers;
}

inline void Mesh::destroy_controller(const Controller* pController)
{
    destroy_component(mControllers, pController);
}

inline void Mesh::destroy_renderer(const Renderer* pRenderer)
{
    destroy_component(mRenderers, pRenderer);
}

void Mesh::update(float deltaTime)
{
    if (mUpdate) {
        mUpdate = false;
        mVertices.clear();
        mIndices.clear();
        if (mspFont && !mText.empty()) {
            glm::vec3 cursor { };
            std::array<Vertex, 4> vertices { };
            std::array<uint16_t, 6> indices { };
            mVertices.reserve(mText.size() * vertices.size());
            mIndices.reserve(mText.size() * indices.size());
            for (size_t i = 0; i < mText.size(); ++i) {
                const auto& glyph = mspFont->get_glyph(mText[i]);
                auto w = glyph.extent.x * 0.5f;
                auto h = glyph.extent.y * 0.5f;
                auto offset = glyph.offset + glm::vec2 { w, h };
                auto kerning = mKerningEnabled && i ? mspFont->get_kerning(mText[i - 1], mText[i]) : 0;

                vertices[0].position = cursor + glm::vec3 { -w + offset.x + kerning,  h - offset.y, 0 };
                vertices[0].position = cursor + glm::vec3 {  w + offset.x + kerning,  h - offset.y, 0 };
                vertices[0].position = cursor + glm::vec3 {  w + offset.x + kerning, -h - offset.y, 0 };
                vertices[0].position = cursor + glm::vec3 { -w + offset.x + kerning, -h - offset.y, 0 };
                cursor[0] += glyph.xAdvance + mGlyphSpacing;

                const auto& atlas = mspFont->get_atlas();
                auto u = glyph.extent.x / atlas.width;
                auto v = glyph.extent.y / atlas.height;
                vertices[0].texcoord = glyph.texcoord + glm::vec2 { -u, -v };
                vertices[1].texcoord = glyph.texcoord + glm::vec2 {  u, -v };
                vertices[2].texcoord = glyph.texcoord + glm::vec2 {  u,  v };
                vertices[3].texcoord = glyph.texcoord + glm::vec2 { -u,  v };

                vertices[0].color = mColor;
                vertices[1].color = mColor;
                vertices[2].color = mColor;
                vertices[3].color = mColor;

                auto vertex_i = (uint16_t)vertices.size();
                indices[0] = vertex_i + 0;
                indices[1] = vertex_i + 1;
                indices[2] = vertex_i + 2;
                indices[3] = vertex_i + 0;
                indices[4] = vertex_i + 2;
                indices[5] = vertex_i + 3;

                for (auto& controller : mControllers) {
                    assert(controller);
                    controller->update(deltaTime, *this, i, vertices);
                }

                mVertices.insert(mVertices.end(), vertices.data(), vertices.data() + vertices.size());
                mIndices.insert(mIndices.end(), indices.data(), indices.data() + indices.size());
            }
        }
        for (auto& controller : mControllers) {
            assert(controller);
            controller->update(deltaTime, *this, mVertices);
        }
        for (auto& renderer : mRenderers) {
            assert(renderer);
            renderer->update(deltaTime, *this);
        }
    }
}

template <typename T>
void Mesh::set_member_value(const T& value, T& member)
{
    if (member != value) {
        member = value;
        mUpdate = true;
    }
}

} // namespace text
} // namespace dst
