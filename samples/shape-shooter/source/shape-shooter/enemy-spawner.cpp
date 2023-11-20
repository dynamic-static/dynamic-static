
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

#include "shape-shooter/enemy-spawner.hpp"
#include "shape-shooter/context.hpp"
#include "shape-shooter/enemy.hpp"

namespace shape_shooter {

void EnemySpawner::update()
{
    auto& context = Context::instance();
    auto& rng = context.rng;
    const auto& pPlayerShip = context.pPlayerShip;
    assert(pPlayerShip);
    auto& entityManager = context.entityManager;
    if (!pPlayerShip->is_dead() && entityManager.get_entity_count() < 200) {
        if (rng.die_roll((int)mInverseSpawnChance) == 1) {
            auto pEnemy = entityManager.create_entity<Enemy>(Sprite::Seeker);
            pEnemy->position = get_spawn_position();
        }
        if (rng.die_roll((int)mInverseSpawnChance) == 1) {
            auto pEnemy = entityManager.create_entity<Enemy>(Sprite::Wanderer);
            pEnemy->position = get_spawn_position();
        }
    }
    if (30 < mInverseSpawnChance) {
        mInverseSpawnChance -= 0.005f;
        // 0.01666f / 0.005f = 3.33333f
        // mInverseSpawnChance -= 3.33333f * deltaTime;
    }
}

void EnemySpawner::reset()
{
    mInverseSpawnChance = 90;
}

glm::vec3 EnemySpawner::get_spawn_position()
{
    glm::vec3 position{ };
    const auto* pPlayerShip = Context::instance().pPlayerShip;
    assert(pPlayerShip);
    do {
        // TODO : Hardcoded values...
        auto& rng = Context::instance().rng;
        position = glm::vec3{ rng.range(-1920.0f, 1920.0f), 0, rng.range(-1080.0f, 1080.0f) };
    } while (glm::distance2(position, pPlayerShip->position) < 250.0f * 250.0f);
    return position;
}

} // namespace shape_shooter
