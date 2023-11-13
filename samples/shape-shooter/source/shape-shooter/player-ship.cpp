
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
    : Entity(Sprite::PlayerShip)
{
}

uint64_t PlayerShip::get_type_id() const
{
    return shape_shooter::get_type_id<PlayerShip>();
}

bool PlayerShip::is_dead() const
{
    return false;
}

void PlayerShip::update(float deltaTime)
{
    auto& context = Context::instance();
    auto& rng = context.rng;
    const auto& playField = context.playField;
    auto& entityManager = context.entityManager;
    const auto& inputManager = context.inputManager;

    auto aimDirection = inputManager.get_aim_direction();
    if (glm::length2(aimDirection) && mCooldownTimer <= 0) {
        mCooldownTimer = mCooldownTime;
        // TODO : Sort out inconsistent rotations from get_orientation()
        // auto bulletOrientation = get_orientation(aimDirection);
        auto aimAngle = std::atan2(aimDirection.z, aimDirection.x);
        auto aimRotation = glm::angleAxis(aimAngle, glm::vec3{ 0, 1, 0 });

        float bulletSpread = rng.range(-0.04f, 0.04f) + rng.range(-0.04f, 0.04f);
        auto bulletVelocity = from_polar(aimAngle + bulletSpread, 11.0f);

        entityManager.create_entity<Bullet>(position + glm::vec3{ 35, 0, -8 } * aimRotation, bulletVelocity);
        entityManager.create_entity<Bullet>(position + glm::vec3{ 35, 0,  8 } * aimRotation, bulletVelocity);
    }
    mCooldownTimer -= deltaTime;

#if 1
    velocity += inputManager.get_movement_direction() * mSpeed * deltaTime;
    position += velocity;
    const auto& halfPlayFieldExtent = playField.extent * 0.5f;
    position = glm::clamp(position, -halfPlayFieldExtent, halfPlayFieldExtent);
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
