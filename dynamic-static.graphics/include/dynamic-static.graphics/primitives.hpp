
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

#include <array>
#include <cassert>
#include <unordered_map>
#include <utility>

namespace dst {
namespace gfx {
namespace primitive {

template <typename IndexType = uint32_t>
using Edge = std::array<IndexType, 2>;

template <typename IndexType = uint32_t>
using Triangle = std::array<IndexType, 3>;

template <typename IndexType = uint32_t>
class EdgeHasher final
{
public:
    inline size_t operator()(const Edge<IndexType>& edge) const
    {
        return std::hash<IndexType> { }(edge[0]) ^ std::hash<IndexType> { }(edge[1]);
    }
};

template <typename IndexType = uint32_t>
Edge<IndexType> create_edge(IndexType i0, IndexType i1)
{
    assert(i0 != i1);
    if (i1 < i0) {
        std::swap(i0, i1);
    }
    return { i0, i1 };
}

template <
    typename IndexType,
    typename ProcessEdgeFunctionType,
    typename ProcessTrianglesFunctionType
>
void subdivide_triangle(
    const Triangle<IndexType>& triangle,
    ProcessEdgeFunctionType processEdge,
    ProcessTrianglesFunctionType processTriangles
)
{
    //               V0
    //              /\
    //             /  \
    //            /    \
    //           /      \
    //          /        \
    //         /          \
    //        /            \
    //       /              \
    //      /                \
    //  V2 /__________________\ V1
    //
    const auto I0 = triangle[0];
    const auto I1 = triangle[1];
    const auto I2 = triangle[2];

    //               V0
    //              /\
    //             /  \
    //            /    \
    //           /      \
    //       V5 /________\ V3
    //         /\        /\
    //        /  \      /  \
    //       /    \    /    \
    //      /      \  /      \
    //  V2 /________\/________\ V1
    //               V4
    const auto I3 = (IndexType)processEdge(create_edge<IndexType>(I0, I1));
    const auto I4 = (IndexType)processEdge(create_edge<IndexType>(I1, I2));
    const auto I5 = (IndexType)processEdge(create_edge<IndexType>(I2, I0));
    processTriangles(
        Triangle<IndexType> { triangle[0], I3, I5 },
        {
            Triangle<IndexType> { I5, I4, I2 },
            Triangle<IndexType> { I5, I3, I4 },
            Triangle<IndexType> { I3, I1, I4 },
        }
    );
}

struct Icosahedron
{
    // FROM : https://gitlab.com/libeigen/eigen/-/blob/master/demos/opengl/icosphere.cpp
    static constexpr float X { 0.525731112119133606f };
    static constexpr float Z { 0.850650808352039932f };
    static constexpr std::array<glm::vec3, 12> Vertices {
        glm::vec3 { -X,  0,  Z }, glm::vec3 {  X,  0,  Z }, glm::vec3 { -X,  0, -Z }, glm::vec3 {  X,  0, -Z },
        glm::vec3 {  0,  Z,  X }, glm::vec3 {  0,  Z, -X }, glm::vec3 {  0, -Z,  X }, glm::vec3 {  0, -Z, -X },
        glm::vec3 {  Z,  X,  0 }, glm::vec3 { -Z,  X,  0 }, glm::vec3 {  Z, -X,  0 }, glm::vec3 { -Z, -X,  0 },
    };
    static constexpr std::array<Triangle<uint32_t>, 20> Triangles {
        Triangle<uint32_t> { 0,  4,  1 }, Triangle<uint32_t> {  0,  9,  4 }, Triangle<uint32_t> {  9,  5,  4 }, Triangle<uint32_t> {  4,  5,  8 }, Triangle<uint32_t> {  4,  8,  1 },
        Triangle<uint32_t> { 8, 10,  1 }, Triangle<uint32_t> {  8,  3, 10 }, Triangle<uint32_t> {  5,  3,  8 }, Triangle<uint32_t> {  5,  2,  3 }, Triangle<uint32_t> {  2,  7,  3 },
        Triangle<uint32_t> { 7, 10,  3 }, Triangle<uint32_t> {  7,  6, 10 }, Triangle<uint32_t> {  7, 11,  6 }, Triangle<uint32_t> { 11,  0,  6 }, Triangle<uint32_t> {  0,  1,  6 },
        Triangle<uint32_t> { 6,  1, 10 }, Triangle<uint32_t> {  9,  0, 11 }, Triangle<uint32_t> {  9, 11,  2 }, Triangle<uint32_t> {  9,  2,  5 }, Triangle<uint32_t> {  7,  2, 11 },
    };
};

struct Cube
{
    static constexpr std::array<glm::vec3, 24> Vertices {
        // Top
        glm::vec3 { -0.5f,  0.5f, -0.5f },
        glm::vec3 {  0.5f,  0.5f, -0.5f },
        glm::vec3 {  0.5f,  0.5f,  0.5f },
        glm::vec3 { -0.5f,  0.5f,  0.5f },
        // Left
        glm::vec3 { -0.5f,  0.5f, -0.5f },
        glm::vec3 { -0.5f,  0.5f,  0.5f },
        glm::vec3 { -0.5f, -0.5f,  0.5f },
        glm::vec3 { -0.5f, -0.5f, -0.5f },
        // Front
        glm::vec3 { -0.5f,  0.5f,  0.5f },
        glm::vec3 {  0.5f,  0.5f,  0.5f },
        glm::vec3 {  0.5f, -0.5f,  0.5f },
        glm::vec3 { -0.5f, -0.5f,  0.5f },
        // Right
        glm::vec3 {  0.5f,  0.5f,  0.5f },
        glm::vec3 {  0.5f,  0.5f, -0.5f },
        glm::vec3 {  0.5f, -0.5f, -0.5f },
        glm::vec3 {  0.5f, -0.5f,  0.5f },
        // Back
        glm::vec3 {  0.5f,  0.5f, -0.5f },
        glm::vec3 { -0.5f,  0.5f, -0.5f },
        glm::vec3 { -0.5f, -0.5f, -0.5f },
        glm::vec3 {  0.5f, -0.5f, -0.5f },
        // Bottom
        glm::vec3 { -0.5f, -0.5f,  0.5f },
        glm::vec3 {  0.5f, -0.5f,  0.5f },
        glm::vec3 {  0.5f, -0.5f, -0.5f },
        glm::vec3 { -0.5f, -0.5f, -0.5f },
    };
    static constexpr std::array<Triangle<uint32_t>, 12> Triangles {
        Triangle<uint32_t> {  0,  1,  2 },
        Triangle<uint32_t> {  2,  3,  0 },
        Triangle<uint32_t> {  4,  5,  6 },
        Triangle<uint32_t> {  6,  7,  4 },
        Triangle<uint32_t> {  8,  9, 10 },
        Triangle<uint32_t> { 10, 11,  8 },
        Triangle<uint32_t> { 12, 13, 14 },
        Triangle<uint32_t> { 14, 15, 12 },
        Triangle<uint32_t> { 16, 17, 18 },
        Triangle<uint32_t> { 18, 19, 16 },
        Triangle<uint32_t> { 20, 21, 22 },
        Triangle<uint32_t> { 22, 23, 20 },
    };
};

} // namespace primitive
} // namespace gfx
} // namespace dst
