
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

#include "gvk/math/camera.hpp"
#include "gvk/math/color.hpp"
#include "gvk/math/transform.hpp"
#include "gvk/system/time.hpp"
#include "gvk/context.hpp"
#include "gvk/defaults.hpp"
#include "gvk/format.hpp"
#include "gvk/handles.hpp"
#include "gvk/mesh.hpp"
#include "gvk/render-target.hpp"
#include "gvk/spir-v.hpp"
#include "gvk/structures.hpp"
#include "gvk/to-string.hpp"

namespace dst {
namespace gfx {

struct EmptyVertex
{
};

struct VertexPositionColor
{
    glm::vec3 position;
    glm::vec4 color;
};

struct VertexPositionTexcoord
{
    glm::vec3 position;
    glm::vec2 texcoord;
};

struct VertexPositionTexcoordColor
{
    glm::vec3 position;
    glm::vec2 texcoord;
    glm::vec4 color;
};

struct VertexPositionNormalColor
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec4 color;
};

struct VertexPositionNormalTexcoordColor
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texcoord;
    glm::vec4 color;
};

} // namespace gfx
} // namespace dst

template <>
inline auto gvk::get_vertex_description<dst::gfx::EmptyVertex>(uint32_t binding)
{
    return gvk::get_vertex_input_attribute_descriptions<>(binding);
}

template <>
inline auto gvk::get_vertex_description<dst::gfx::VertexPositionColor>(uint32_t binding)
{
    return gvk::get_vertex_input_attribute_descriptions<
        glm::vec3,
        glm::vec4
    >(binding);
}

template <>
inline auto gvk::get_vertex_description<dst::gfx::VertexPositionTexcoord>(uint32_t binding)
{
    return gvk::get_vertex_input_attribute_descriptions<
        glm::vec3,
        glm::vec2
    >(binding);
}

template <>
inline auto gvk::get_vertex_description<dst::gfx::VertexPositionNormalColor>(uint32_t binding)
{
    return gvk::get_vertex_input_attribute_descriptions<
        glm::vec3,
        glm::vec3,
        glm::vec4
    >(binding);
}

template <>
inline auto gvk::get_vertex_description<dst::gfx::VertexPositionTexcoordColor>(uint32_t binding)
{
    return gvk::get_vertex_input_attribute_descriptions<
        glm::vec3,
        glm::vec2,
        glm::vec4
    >(binding);
}
