
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

#include <utility>

namespace shape_shooter {

template <typename T>
class RingBuffer final
{
public:
    using reference = std::vector<T>::reference;
    using const_reference = std::vector<T>::const_reference;

    RingBuffer() = default;

    inline RingBuffer(size_t capacity)
    {
        resize(capacity);
    }

    inline const T& operator[](size_t index) const
    {
        return mElements[(mOffset + index) % mElements.size()];
    }

    inline T& operator[](size_t index)
    {
        return const_cast<T&>(const_cast<const RingBuffer<T>*>(this)->operator[](index));
    }

    inline void clear()
    {
        mCount = 0;
        mOffset = 0;
        mElements.clear();
    }

    inline void resize(size_t capacity)
    {
        clear();
        mElements.resize(capacity);
    }

    inline size_t count() const
    {
        return mCount;
    }

    inline size_t capacity() const
    {
        return mElements.size();
    }

    inline const_reference back() const
    {
        assert(mCount);
        assert(mElements.size());
        return operator[](mCount - 1);
    }

    inline reference back()
    {
        return const_cast<T&>(const_cast<const RingBuffer<T>*>(this)->back());
    }

    inline void push_back(const T& value)
    {
        if (mCount == mElements.size()) {
            operator[](0) = value;
            mOffset = (mOffset + 1) % mElements.size();
        } else {
            operator[](mCount++) = value;
        }
    }

    inline void pop_back()
    {
        assert(mCount);
        --mCount;
    }

private:
    size_t mCount{ };
    size_t mOffset{ };
    std::vector<T> mElements;
};

} // namespace shape_shooter
