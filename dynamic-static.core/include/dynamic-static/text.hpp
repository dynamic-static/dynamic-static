
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
#include <functional>
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

    static void create(const char* pFilePath, const char* pCharacterSet, float size, std::shared_ptr<Font>* pspFont);
    float get_ascent() const;
    float get_descent() const;
    float get_line_height() const;
    float get_base_line() const;
    const Atlas& get_atlas() const;
    const Glyph& get_glyph(int32_t codepoint) const;
    float get_kerning(int32_t lhsCodepoint, int32_t rhsCodepoint) const;

private:
    float mAscent { };
    float mDescent { };
    float mLineHeight { };
    float mBaseLine { };
    Atlas mAtlas;
    std::unordered_map<int32_t, Glyph> mGlyphs;
    std::map<std::pair<int32_t, int32_t>, float> mKerningPairs;
};

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
        virtual ~Controller() = 0;
        virtual void update(float deltaTime, const Mesh& mesh, size_t i, std::array<Vertex, 4>& vertices);
        virtual void update(float deltaTime, const Mesh& mesh, std::vector<Vertex>& vertices);
    };

    class Renderer
    {
    public:
        virtual ~Renderer() = 0;
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
    void destroy_controller(const Controller* pController);
    void destroy_renderer(const Renderer* pRenderer);
    void update(float deltaTime);

    template <typename ControllerType, typename CreateControllerFunctionType>
    inline auto create_controller(CreateControllerFunctionType createController)
    {
        return create_component<ControllerType>(mControllers, createController);
    }

    template <typename RendererType, typename CreateRendererFunctionType>
    inline auto create_renderer(CreateRendererFunctionType createRenderer)
    {
        return create_component<RendererType>(mRenderers, createRenderer);
    }

private:
    template <typename T>
    void set_member_value(const T& value, T& member);

    template <typename ComponentType, typename CollectionType, typename CreateComponentFunctionType>
    inline auto create_component(CollectionType& collection, CreateComponentFunctionType createComponent)
    {
        mUpdate = true;
        collection.push_back(std::make_unique<ComponentType>());
        // collection.emplace_back(std::make_unique<ComponentType>());
        return createComponent(const_cast<const Mesh&>(*this), (ComponentType&)*collection.back());
    }

    template <typename ComponentType, typename CollectionType>
    inline void destroy_component(CollectionType& collection, const ComponentType* pComponent)
    {
        auto predicate = [&](const auto& upComponent) { return upComponent.get() == pComponent; };
        auto itr = std::find_if(collection.begin(), collection.end(), predicate);
        if (itr != collection.end()) {
            collection.erase(itr);
            mUpdate = true;
        }
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
