
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

#include "shape-shooter/enemy.hpp"
#include "shape-shooter/context.hpp"
#include "shape-shooter/utilities.hpp"

namespace shape_shooter {

Enemy::Enemy(Sprite sprite)
    : Entity(sprite)
{
    switch (mSprite) {
    case Sprite::Wanderer: {
        mBehaviors.emplace_back(std::make_unique<MoveRandomly>());
        mPointValue = 1;
    } break;
    case Sprite::Seeker: {
        mBehaviors.emplace_back(std::make_unique<FollowPlayer>());
        mPointValue = 2;
    } break;
    default: {
        assert(false);
    }
    }
    radius = get_sprite_extent().x * 0.5f;
    color = gvk::math::Color::Transparent;
}

uint64_t Enemy::get_type_id() const
{
    return shape_shooter::get_type_id<Enemy>();
}

uint32_t Enemy::get_point_value() const
{
    return mPointValue;
}

bool Enemy::is_active() const
{
    return mTimeUntilStart <= 0;
}

void Enemy::handle_collision(const Enemy& other)
{
    auto direction = position - other.position;
    velocity += 10.0f * direction / (glm::length2(direction) + 1);
}

void Enemy::update()
{
    auto deltaTime = Context::instance().clock.elapsed<gvk::system::Seconds<float>>();
    if (is_active()) {
        for (const auto& upBehavior : mBehaviors) {
            assert(upBehavior);
            upBehavior->update(*this);
        }
    } else {
        mTimeUntilStart = glm::max(0.0f, mTimeUntilStart - deltaTime);
        color = gvk::math::Color::White * (1.0f - mTimeUntilStart);
    }
    // TODO : Hardcoded values...
    position += velocity;
    const auto& halfPlayFieldExtent = Context::instance().playField.extent * 0.5f;
    position = glm::clamp(position, -halfPlayFieldExtent, halfPlayFieldExtent);
    velocity *= 0.8f;
}

void Enemy::draw() const
{
    Entity::draw();
}

void Enemy::FollowPlayer::update(Enemy& enemy)
{
    (void)enemy;
#if 1
    const auto& context = Context::instance();
    assert(context.pPlayerShip);
    const auto& playerShip = *context.pPlayerShip;
    if (!playerShip.is_dead()) {
        auto direction = playerShip.position - enemy.position;
        if (direction.x || direction.y || direction.z) {
            direction = glm::normalize(direction);
        }
        enemy.velocity += direction * mAcceleration;
    }
    if (enemy.velocity.x || enemy.velocity.y) {
        enemy.orientation = std::atan2(-enemy.velocity.z, enemy.velocity.x);
    }
#endif
}

Enemy::MoveRandomly::MoveRandomly()
    : mDirection{ Context::instance().rng.range(0.0f, glm::two_pi<float>()) }
{
}

void Enemy::MoveRandomly::update(Enemy& enemy)
{
    (void)enemy;
#if 1
    // TODO : Hardcoded values...
    auto& context = Context::instance();
    auto& rng = context.rng;
    if (!mUpdateCounter) {
        mDirection += rng.range(-0.1f, 0.1f);
        mDirection = glm::wrapAngle(mDirection);
    }
    enemy.velocity += from_polar(mDirection, 0.4f);
    enemy.orientation -= 0.05f;
    if (!context.playField.contains(enemy.position)) {
        mDirection = std::atan2(-enemy.position.z, -enemy.position.x) + rng.range(-glm::half_pi<float>(), glm::half_pi<float>());
    }
#if 0
    if (enemy.position.x <= -context.playField.x * 0.5f || context.playField.x * 0.5f <= enemy.position.x ||
        enemy.position.z <= -context.playField.y * 0.5f || context.playField.y * 0.5f <= enemy.position.z) {
        mDirection = std::atan2(-enemy.position.z, -enemy.position.x) + rng.range(-glm::half_pi<float>(), glm::half_pi<float>());
    }
#endif
    if (6 <= mUpdateCounter++) {
        mUpdateCounter = 0;
    }
#endif
}

} // namespace shape_shooter
