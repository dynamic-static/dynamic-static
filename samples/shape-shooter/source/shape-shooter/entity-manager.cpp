
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

#include "shape-shooter/entity-manager.hpp"
#include "shape-shooter/context.hpp"

#include <vector>

namespace shape_shooter {

uint32_t EntityManager::get_entity_count() const
{
    return (uint32_t)mEntities.size();
}

void EntityManager::update(float deltaTime)
{
    mUpdating = true;
    handle_collisions();
    for (auto& upEntity : mEntities) {
        assert(upEntity);
        upEntity->update(deltaTime);
    }
    mUpdating = false;
    for (auto& upAddedEntity : mAddedEntities) {
        assert(upAddedEntity);
        add_entity(std::move(upAddedEntity));
    }
    mAddedEntities.clear();
    std::erase_if(mBullets, [](const Bullet* pBullet) { assert(pBullet); return pBullet->expired; });
    std::erase_if(mEnemies, [](const Enemy* pEnemy) { assert(pEnemy); return pEnemy->expired; });
    // Blackholes
    std::erase_if(mEntities, [](const std::unique_ptr<Entity>& upEntity) { assert(upEntity); return upEntity->expired; });
}

void EntityManager::handle_collisions()
{
    for (uint32_t i = 0; i < mEnemies.size(); ++i) {
        assert(mEnemies[i]);
        for (uint32_t j = i + 1; j < mEnemies.size(); ++j) {
            assert(mEnemies[j]);
            if (Entity::collision(*mEnemies[i], *mEnemies[j])) {
                mEnemies[i]->handle_collision(*mEnemies[j]);
                mEnemies[j]->handle_collision(*mEnemies[i]);
            }
        }
    }
    for (auto pEnemy : mEnemies) {
        assert(pEnemy);
        for (auto pBullet : mBullets) {
            assert(pBullet);
            if (Entity::collision(*pEnemy, *pBullet)) {
                Context::instance().scoreBoard.add_points(pEnemy->get_point_value());
                Context::instance().scoreBoard.increase_multiplier();
                pEnemy->expired = true;
                pBullet->expired = true;
            }
        }
    }
    const auto& pPlayerShip = Context::instance().pPlayerShip;
    for (auto pEnemy : mEnemies) {
        assert(pEnemy);
        if (pEnemy->is_active() && Entity::collision(*pEnemy, *pPlayerShip)) {
            kill_player();
            break;
        }
    }
}

void EntityManager::kill_player()
{

}

void EntityManager::draw(dst::gfx::SpriteRenderer& spriteRenderer) const
{
    for (const auto& entity : mEntities) {
        entity->draw(spriteRenderer);
    }
}

} // namespace shape_shooter
