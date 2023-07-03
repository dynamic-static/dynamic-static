
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

#include "dynamic-static.graphics/buffer.hpp"

namespace dst {
namespace gfx {

VkResult ResizableBuffer::create(const gvk::Device& device, const VkBufferCreateInfo* pCreateInfo, const VmaAllocationCreateInfo* pAllocationCreateInfo, ResizableBuffer* pResizableBuffer)
{
    assert(pCreateInfo);
    assert(pAllocationCreateInfo);
    assert(pResizableBuffer);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(gvk::Buffer::create(device, pCreateInfo, pAllocationCreateInfo, &pResizableBuffer->mBuffer));
        pResizableBuffer->mSize = pCreateInfo->size;
        pResizableBuffer->mAllocationCreateInfo = *pAllocationCreateInfo;
    } gvk_result_scope_end;
    return gvkResult;
}

const gvk::Buffer& ResizableBuffer::buffer() const
{
    return mBuffer;
}

VkBool32 ResizableBuffer::empty() const
{
    return !mBuffer;
}

VkDeviceSize ResizableBuffer::size() const
{
    return mSize;
}

VkResult ResizableBuffer::resize(VkDeviceSize size)
{
    assert(mBuffer);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        auto bufferCreateInfo = mBuffer.get<VkBufferCreateInfo>();
        if (bufferCreateInfo.size < size) {
            bufferCreateInfo.size = size;
            const auto& device = mBuffer.get<gvk::Device>();
            gvk_result(gvk::Buffer::create(device, &bufferCreateInfo, &mAllocationCreateInfo, &mBuffer));
            mSize = bufferCreateInfo.size;
        }
    } gvk_result_scope_end;
    return gvkResult;
}

void ResizableBuffer::clear()
{
    mBuffer.reset();
}

} // namespace gfx
} // namespace dst
