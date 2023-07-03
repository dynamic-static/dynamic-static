
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

#include "dynamic-static/bin-pack.hpp"
#include "dynamic-static/defines.hpp"
#include "dynamic-static/filesystem.hpp"
#include "dynamic-static/image.hpp"

#include "glm/glm.hpp"
#include "stb/stb_truetype.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace dst {
namespace text {

inline bool is_printable(int32_t codepoint);

struct Glyph
{
    static constexpr int32_t           InvalidId            { -1 };
    static constexpr int32_t           NullId               { 0 };    // &#00   U+0000
    static constexpr int32_t           LineFeedId           { 10 };   // &#10   U+000A
    static constexpr int32_t           SpaceId              { 32 };   // &#32   U+0020
    static constexpr int32_t           UnderscoreId         { 95 };   // &#95   U+005F
    static constexpr int32_t           LeftToRightId        { 8206 }; // &#8206 U+200E
    static constexpr int32_t           RightToLeftId        { 8207 }; // &#8207 U+200F
    static constexpr int32_t           LineSeperatorId      { 8232 }; // &#8232 U+
    static constexpr int32_t           ParagraphSeperatorId { 8233 }; // &#8233 U+
    static constexpr int32_t           NullInputId          { 9618 }; // &#9618 U+2592
    static constexpr const char* const LineFeed             { "\n" };

    friend bool operator==(const Glyph& lhs, const Glyph& rhs);
    friend bool operator!=(const Glyph& lhs, const Glyph& rhs);
    friend bool operator<(const Glyph& lhs, const Glyph& rhs);
    friend bool operator>(const Glyph& lhs, const Glyph& rhs);
    friend bool operator<=(const Glyph& lhs, const Glyph& rhs);
    friend bool operator>=(const Glyph& lhs, const Glyph& rhs);

    int32_t codepoint { };
    int32_t page { };
    glm::vec2 texcoord { };
    glm::vec2 extent { };
    glm::vec2 offset { };
    float xAdvance { };
};

class Atlas final
{
public:
    uint32_t width { };
    uint32_t height { };
    std::vector<Image<R8G8B8A8Unorm>> pages;
};

class Font final
{
public:
    static constexpr const char* const DefaultCharacterSet {
        "`~!@#$%^&*()-=_+[]\\{}|;':\",./<>? "
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789"
    };

    inline static void create(const char* pFilePath, const char* pCharacterSet, std::shared_ptr<Font>* pspFont)
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
            auto scale = stbtt_ScaleForPixelHeight(&fontInfo, 64);
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

    inline float get_ascent() const { return mAscent; }
    inline float get_descent() const { return mDescent; }
    inline float get_line_height() const { return mLineHeight; }
    inline float get_base_line() const { return mBaseLine; }
    inline const Atlas& get_atlas() const { return mAtlas; }

    inline const Glyph& get_glyph(int32_t codepoint) const
    {
        static const Glyph NullGlyph;
        auto itr = mGlyphs.find(codepoint);
        return itr != mGlyphs.end() ? itr->second : NullGlyph;
    }

    inline float get_kerning(int32_t lhsCodepoint, int32_t rhsCodepoint) const
    {
        auto itr = mKerningPairs.find({ lhsCodepoint, rhsCodepoint });
        return itr != mKerningPairs.end() ? itr->second : 0;
    }

private:
    float mAscent { };
    float mDescent { };
    float mLineHeight { };
    float mBaseLine { };
    Atlas mAtlas;
    std::unordered_map<int32_t, Glyph> mGlyphs;
    std::map<std::pair<int32_t, int32_t>, float> mKerningPairs;
};

#if 0
class Layout final
{
public:
    template <typename AtlasType, typename ProcessGlyphFunctionType>
    inline void update(const Font& font, const char* pText, ProcessGlyphFunctionType processGlyph)
    {
        using vec2 = std::array<float, 2>;
        auto add = [](const vec2& lhs, const vec2& rhs) { return vec2 { lhs[0] + rhs[0], lhs[1] + rhs[1] }; };
        auto sub = [](const vec2& lhs, const vec2& rhs) { return vec2 { lhs[0] - rhs[0], lhs[1] - rhs[1] }; };
        auto mul = [](const vec2& lhs, const vec2& rhs) { return vec2 { lhs[0] * rhs[0], lhs[1] * rhs[1] }; };

        vec2 scale { 1, 1 };

        vec2 cursor { };
        std::array<vec2, 4> vertices { };
        std::array<vec2, 4> texcoords { };
        std::array<uint16_t, 6> indices { };

        char* pPrev = nullptr;
        while (*pText) {
            if (pPrev) {

            }

            const auto& glyph = font.get_glyph();
            auto w = glyph.width * 0.5f;
            auto h = glyph.height * 0.5f;
            auto xOffset = glyph.xOffset + w;
            auto yOffset = glyph.yOffset + h;
#if 0
            vec2 extent { glyph.width * 0.5f, glyph.height * 0.5f };
            vec2 offset { glyph.xOffset + extent[0], glyph.yOffset + extent[1] };
#endif
            vertices[0] = add(cursor, mul({ -w + xOffset,  h - yOffset }, scale));
            vertices[1] = add(cursor, mul({  w + xOffset,  h - yOffset }, scale));
            vertices[2] = add(cursor, mul({  w + xOffset, -h - yOffset }, scale));
            vertices[3] = add(cursor, mul({ -w + xOffset, -h - yOffset }, scale));
            float kerning = 0;

            ++pText;
        }
    }
};
#endif

class Mesh final
{
public:
    struct Vertex
    {
        glm::vec3 position { };
        glm::vec2 texcoord { };
        glm::vec4 color { };
    };

    class Controller
    {
    public:
        virtual void update(float deltaTime, const Mesh& mesh, size_t i, std::array<Vertex, 4>& vertices);
        virtual void update(float deltaTime, const Mesh& mesh, std::vector<Vertex>& vertices);
    };

    class Renderer
    {
    public:
        virtual void update(float deltaTime, const Mesh& mesh);
    };

    const std::string& get_text() const;
    void set_text(const std::string& text);
    const std::shared_ptr<Font>& get_font() const;
    void set_font(const std::shared_ptr<Font>& spFont);
    const std::vector<Vertex>& get_vertices() const;
    const std::vector<uint16_t>& get_indices() const;
    float get_glyph_spacing() const;
    void set_glpyh_spacing(float glyphSpacing);
    float get_line_spacing() const;
    void set_line_spacing(float lineSpacing);
    bool kerning_enabled() const;
    void kerning_enabled(bool kerningEnabled);
    const glm::vec4& get_color() const;
    void set_color(const glm::vec4& color);
    const std::vector<std::unique_ptr<Controller>>& get_controllers() const;
    const std::vector<std::unique_ptr<Renderer>>& get_renderers() const;

    template <typename ControllerType, typename ReturnType, typename ...ControllerCreateArgsTypes>
    inline ReturnType create_controller(ControllerCreateArgsTypes&&... args)
    {
        return create_component(mControllers, std::forward<ControllerCreateArgsTypes>(args)...);
    }

    inline void destroy_controller(const Controller* pController)
    {
        destroy_component(mControllers, pController);
    }

    template <typename RendererType, typename ReturnType, typename ...RenderCreateArgsTypes>
    inline ReturnType create_renderer(RenderCreateArgsTypes&&... args)
    {
        return create_component(mRenderers, std::forward<RenderCreateArgsTypes>(args)...);
    }

    inline void destroy_renderer(const Renderer* pRenderer)
    {
        destroy_component(mRenderers, pRenderer);
    }

    void update(float deltaTime);

private:
    template <typename T>
    void set_member_value(const T& value, T& member);

    template <typename CollectionType, typename ComponentType, typename ...ComponentCreateArgsTypes>
    inline void create_component(CollectionType& collection, ComponentCreateArgsTypes&&... args)
    {
        mUpdate = true;
        auto pComponent = collection.emplace_back(std::make_unique<ComponentType>()).get();
        return ComponentType::create(std::forward<ComponentCreateArgsTypes>(args)..., pComponent);
    }

    template <typename CollectionType, typename ComponentType>
    inline void destroy_component(CollectionType& collection, const ComponentType* pComponent)
    {
        (void)collection;
        (void)pComponent;
#if 0
        auto itr = std::find_if(collection.begin(), collection.end(), [&](auto component) { return component.get() == pComponent; });
        if (itr != collection.end()) {
            collection.erase(itr);
            mUpdate = true;
        }
#endif
    }

    std::string mText;
    std::shared_ptr<Font> mspFont;
    std::vector<Vertex> mVertices;
    std::vector<uint16_t> mIndices;
    float mGlyphSpacing { 0 };
    float mLineSpacing { 0 };
    bool mKerningEnabled { true };
    glm::vec4 mColor { 1, 1, 1, 1 };
    std::vector<std::unique_ptr<Controller>> mControllers;
    std::vector<std::unique_ptr<Renderer>> mRenderers;
    bool mUpdate { true };
};

} // namespace text
} // namespace dst
