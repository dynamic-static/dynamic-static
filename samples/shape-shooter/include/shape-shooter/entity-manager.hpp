
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
#include "shape-shooter/entity.hpp"
#include "shape-shooter/input-manager.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace shape_shooter {

class EntityManager final
{
public:
    EntityManager() = default;

    uint32_t get_entity_count() const;
    void update(float deltaTime);
    void handle_collisions();
    void kill_player();
    void draw(dst::gfx::SpriteRenderer& spriteRenderer) const;

    template <typename EntityType, typename ...Args>
    inline EntityType* create_entity(Args&&... args)
    {
        auto upEntity = std::make_unique<EntityType>(std::forward<Args>(args)...);
        auto pEntity = upEntity.get();
        if (!mUpdating) {
            add_entity(std::move(upEntity));
        } else {
            mAddedEntities.emplace_back(std::move(upEntity));
        }
        return pEntity;
    }

private:
    template <typename EntityType>
    inline void add_entity(std::unique_ptr<EntityType>&& upEntity)
    {
        assert(upEntity);
        mEntities.emplace_back(std::move(upEntity));
    }

    std::vector<std::unique_ptr<Entity>> mEntities;
    std::vector<std::unique_ptr<Entity>> mAddedEntities;
    bool mUpdating{ };

    EntityManager(const EntityManager&) = delete;
    EntityManager& operator=(const EntityManager&) = delete;
};

} // namespace shape_shooter
