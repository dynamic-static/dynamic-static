
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

#include "dynamic-static/defines.hpp"

#include "stb/stb_image.h"
#include "stb/stb_image_write.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstring>
#include <fstream>
#include <vector>

namespace dst {

using R8G8B8A8Unorm = std::array<uint8_t, 4>;

using Extent3D = std::array<uint32_t, 3>;

template <typename TexelType = R8G8B8A8Unorm>
class Image final
{
public:
    using Texel = TexelType;

    Image(const Extent3D& extent = { }, const uint8_t* pData = nullptr)
    {
        resize(extent, pData);
    }

    inline const TexelType& operator[](const Extent3D& texcoord) const
    {
        assert(texcoord[0] < mExtent[0]);
        assert(texcoord[1] < mExtent[1]);
        assert(texcoord[2] < mExtent[2]);
        return mTexels[texcoord[2] * mExtent[0] * mExtent[1] + texcoord[1] * mExtent[0] + texcoord[0]];
    }

    inline TexelType& operator[](const Extent3D& texcoord)
    {
        auto pConstThis = const_cast<const Image<TexelType>*>(this);
        return const_cast<TexelType&>((*pConstThis)[texcoord]);
    }

    bool empty() const
    {
        return mTexels.empty();
    }

    const TexelType* data() const
    {
        return mTexels.data();
    }

    TexelType* data()
    {
        return mTexels.data();
    }

    inline const Extent3D& get_extent() const
    {
        return mExtent;
    }

    inline size_t size_bytes() const
    {
        return mTexels.size() * sizeof(TexelType);
    }

    inline size_t size() const
    {
        return mTexels.size();
    }

    inline void clear()
    {
        mExtent = { };
        mTexels.clear();
    }

    inline void resize(const Extent3D& extent, const uint8_t* pData = nullptr)
    {
        clear();
        mExtent = extent;
        if (mExtent[0] || mExtent[1] || mExtent[2]) {
            mExtent[0] = std::max(1u, extent[0]);
            mExtent[1] = std::max(1u, extent[1]);
            mExtent[2] = std::max(1u, extent[2]);
            auto texelCount = mExtent[0] * mExtent[1] * mExtent[2];
            if (pData) {
                auto pTexels = (const TexelType*)pData;
                mTexels.insert(mTexels.end(), pTexels, pTexels + texelCount);
            } else {
                mTexels.resize(texelCount);
            }
        }
    }

private:
    Extent3D mExtent { };
    std::vector<TexelType> mTexels { };
};

inline bool save_png(const char* pFilePath, const Image<>& image)
{
    assert(pFilePath);
    assert(!image.empty());
    auto stride = sizeof(Image<>::Texel) * image.get_extent()[0];
    return stbi_write_png(pFilePath, (int)image.get_extent()[0], (int)image.get_extent()[1], 4, image.data(), (int)stride);
}

inline bool load_png(const char* pFilePath, Image<>* pImage)
{
    assert(pFilePath);
    assert(pImage);
    pImage->clear();
    int width = 0;
    int height = 0;
    int components = 0;
    auto pImageData = stbi_load(pFilePath, &width, &height, &components, STBI_rgb_alpha);
    if (pImageData) {
        pImage->resize({ (uint32_t)width, (uint32_t)height }, pImageData);
        stbi_image_free(pImageData);
    }
    return !pImage->empty();
}

} // namespace dst
