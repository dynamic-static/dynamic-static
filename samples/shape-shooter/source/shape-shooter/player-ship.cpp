
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

#include "shape-shooter/player-ship.hpp"
#include "shape-shooter/bullet.hpp"
#include "shape-shooter/context.hpp"
#include "shape-shooter/utilities.hpp"

namespace shape_shooter {

PlayerShip::PlayerShip()
{
}

bool PlayerShip::is_dead() const
{
    return false;
}

void PlayerShip::update(const InputManager& inputManager, float deltaTime)
{
    (void)inputManager;
    (void)deltaTime;

    auto aim = glm::vec3(1, 0, 0);
    if (glm::length2(aim) && mCooldownTimer <= 0) {
        mCooldownTimer = mCooldownTime;
        auto bulletOrientation = get_orientation(aim);
        float randomSpread = 0; // TODO :
        // auto bulletVelocity = from_polar(bulletOrientation + randomSpread, 11.0f);
        auto bulletVelocity = from_polar(bulletOrientation + randomSpread, 0.18f);
        auto offset = glm::vec3(0); // TODO :
        Context::instance().entityManager.create_entity<Bullet>(position + offset, bulletVelocity);
    }
    mCooldownTimer -= deltaTime;

#if 1
    velocity += inputManager.get_movement_direction() * mSpeed * deltaTime;
    position += velocity;
    // TODO : Clamp position to playfield
    if (glm::length2(velocity)) {
        orientation = get_orientation(velocity);
    }
    // TODO : make_exhaust_fire();
    velocity = { };
#endif
}

void PlayerShip::draw(dst::gfx::SpriteRenderer& spriteRenderer) const
{
    if (!is_dead()) {
        Entity::draw(spriteRenderer);
    }
}

} // namespace shape_shooter
