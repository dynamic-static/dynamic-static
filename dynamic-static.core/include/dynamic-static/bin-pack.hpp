
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

#include <algorithm>
#include <memory>
#include <vector>

namespace dst {

struct BinPackCell
{
    inline int top() const { return y; }
    inline int left() const { return x; }
    inline int right() const { return x + width; }
    inline int bottom() const { return y + height; }
    inline int area() const { return width * height; }

    int x { };
    int y { };
    int width { };
    int height { };
};

template <typename T>
struct BinPackEntry
{
    BinPackCell cell { };
    int page { };
    T value { };
};

struct BinPackInfo
{
    int width { 1024 };
    int height { 1024 };
    int pageCount { 1 };
    int padding { 2 };
};

namespace detail {

class BinPackNode final
{
public:
    template <typename T>
    inline BinPackNode* insert(BinPackEntry<T>& entry)
    {
        assert((pLeft == nullptr) == (pRight == nullptr));
        BinPackNode* pNode = nullptr;
        if (pLeft) {
            auto insertLeft = pLeft->cell.area() < pRight->cell.area();
            pNode = insertLeft ? pLeft->insert(entry) : pRight->insert(entry);
            if (!pNode) {
                insertLeft ? pRight->insert(entry) : pLeft->insert(entry);
            }
        } else if (!occupied && entry.cell.width <= cell.width && entry.cell.height <= cell.height) {
            pLeft = std::make_unique<BinPackNode>();
            pRight = std::make_unique<BinPackNode>();

            // TODO : Split policy...
            pLeft->cell.x = cell.left();
            pLeft->cell.y = cell.top() + entry.cell.height;
            pLeft->cell.width = cell.width;
            pLeft->cell.height = cell.height - entry.cell.height;

            pRight->cell.x = cell.left() + entry.cell.width;
            pRight->cell.y = cell.top();
            pRight->cell.width = cell.width - entry.cell.width;
            pRight->cell.height = entry.cell.height;

            entry.cell.x = cell.x;
            entry.cell.y = cell.y;
            occupied = true;
            pNode = this;
        }
        return pNode;
    }

    BinPackCell cell { };
    std::unique_ptr<BinPackNode> pLeft { };
    std::unique_ptr<BinPackNode> pRight { };
    std::unique_ptr<BinPackNode> pNextPage { };
    bool occupied { };
};

} // namespace detail

template <typename T>
inline void bin_pack(BinPackInfo* pInfo, size_t entryCount, BinPackEntry<T>* pEntries)
{
    assert(pInfo);
    assert(entryCount);
    assert(pEntries);

    // TODO : Sort policy
    std::sort(
        pEntries, pEntries + entryCount,
        [](const BinPackEntry<T>& lhs, const BinPackEntry<T>& rhs)
        {
            return
                lhs.cell.height == rhs.cell.height ?
                lhs.cell.width > rhs.cell.width :
                lhs.cell.height > rhs.cell.height;
        }
    );

    // Apply padding
    auto padding = pInfo->padding * 2;
    for (size_t i = 0; i < entryCount; ++i) {
        pEntries[i].cell.width += padding;
        pEntries[i].cell.height += padding;
    }

    // Create root node and insert each entry into the tree
    auto pRoot = std::make_unique<detail::BinPackNode>();
    pRoot->cell.width = pInfo->width;
    pRoot->cell.height = pInfo->height;
    for (size_t i = 0; i < entryCount; ++i) {
        auto& entry = pEntries[i];
        auto pPage = pRoot.get();
        while (!pPage->insert(entry)) {
            if (!pPage->pNextPage) {
                pPage->pNextPage = std::make_unique<detail::BinPackNode>();
                pPage->pNextPage->cell.width = pInfo->width;
                pPage->pNextPage->cell.height = pInfo->height;
            }
            pPage = pPage->pNextPage.get();
            ++entry.page;
        }
        pInfo->pageCount = std::max(pInfo->pageCount, entry.page + 1);
#if 0
        if (!pRoot->insert(entry)) {
            auto pPage = pRoot.get();
            while (pPage->pNextPage) {
                pPage = pPage->pNextPage;
                
            }

            auto pNode = pRoot.get();
            while (pNode->pNextPage) {
                pNode = pNode->pNextPage.get();
                ++entry.page;
            }
            if (!pNode->insert(entry)) {
                // TODO : Auto grow to limit
                pNode->pNextPage = std::make_unique<detail::BinPackNode>();
                pNode->pNextPage->cell.width = pInfo->width;
                pNode->pNextPage->cell.height = pInfo->height;
                pNode->pNextPage->insert(entry);
                ++entry.page;
            }
            pInfo->pageCount = std::max(pInfo->pageCount, entry.page + 1);
        }
#endif
    }

    // Remove padding
    for (size_t i = 0; i < entryCount; ++i) {
        pEntries[i].cell.width -= padding;
        pEntries[i].cell.height -= padding;
    }
}

} // namespace dst
