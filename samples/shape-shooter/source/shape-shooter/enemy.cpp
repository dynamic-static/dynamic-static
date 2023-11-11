
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

uint32_t Enemy::get_point_value() const
{
    return mPointValue;
}

bool Enemy::is_active() const
{
    return mTimeUntilStart <= 0;
}

void Enemy::update(float deltaTime)
{
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
    position.x = glm::clamp(position.x, -1920 * 0.5f, 1920 * 0.5f);
    position.z = glm::clamp(position.z, -1080 * 0.5f, 1080 * 0.5f);
    velocity *= 0.8f;
}

void Enemy::draw(dst::gfx::SpriteRenderer& spriteRenderer) const
{
    Entity::draw(spriteRenderer);
}

void Enemy::FollowPlayer::update(Enemy& enemy)
{
    (void)enemy;
}

Enemy::MoveRandomly::MoveRandomly()
    : mDirection{ Context::instance().rng.range(0.0f, glm::two_pi<float>()) }
{
}

void Enemy::MoveRandomly::update(Enemy& enemy)
{
    // TODO : Hardcoded values...
    auto& context = Context::instance();
    if (!mUpdateCounter) {
        mDirection += context.rng.range(-0.1f, 0.1f);
        mDirection = glm::wrapAngle(mDirection);
    }
    enemy.velocity += from_polar(mDirection, 0.4f);
    enemy.orientation -= 0.05f;
    // TODO : Make enemy move away from edge...
    if (6 <= mUpdateCounter++) {
        mUpdateCounter = 0;
    }
}

} // namespace shape_shooter
