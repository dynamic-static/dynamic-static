
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

void PlayerShip::update()
{
    auto& context = Context::instance();
    auto& rng = context.rng;
    const auto& playField = context.playField;
    auto& entityManager = context.entityManager;
    (void)entityManager;
    const auto& inputManager = context.inputManager;
    auto deltaTime = Context::instance().clock.elapsed<gvk::system::Seconds<float>>();

    auto aimDirection = inputManager.get_aim_direction();
    if (glm::length2(aimDirection) && mCooldownTimer <= 0) {
        mCooldownTimer = mCooldownTime;
        // TODO : Sort out inconsistent rotations from get_orientation()
        // auto bulletOrientation = get_orientation(aimDirection);
        auto aimAngle = std::atan2(aimDirection.z, aimDirection.x);
        auto aimRotation = glm::angleAxis(aimAngle, glm::vec3{ 0, 1, 0 });
        (void)aimRotation;

        float bulletSpread = rng.range(-0.04f, 0.04f) + rng.range(-0.04f, 0.04f);
        auto bulletVelocity = from_polar(aimAngle + bulletSpread, 11.0f);
        (void)bulletVelocity;

#if 0 // Disabled to debug grid
        entityManager.create_entity<Bullet>(position + glm::vec3{ 35, 0, -8 } * aimRotation, bulletVelocity);
        entityManager.create_entity<Bullet>(position + glm::vec3{ 35, 0,  8 } * aimRotation, bulletVelocity);
#endif
    }
    mCooldownTimer -= deltaTime;

#if 1
    velocity += inputManager.get_movement_direction() * mSpeed * deltaTime;
    position += velocity;
    const auto& halfPlayFieldExtent = playField.extent * 0.5f;
    position = glm::clamp(position, -halfPlayFieldExtent, halfPlayFieldExtent);
    if (0.0f < glm::length2(velocity)) {
        orientation = get_orientation(velocity);
    }
    make_exhaust_fire();
    velocity = { };
#endif
}

void PlayerShip::draw() const
{
    if (!is_dead()) {
        Entity::draw();
    }
}

void PlayerShip::make_exhaust_fire()
{
    auto totalTime = Context::instance().clock.total<gvk::system::Seconds<float>>();
    if (0.1f < glm::length2(velocity)) {
        // TODO : Documentation
        orientation = get_orientation(velocity);
        auto particleRotation = glm::angleAxis(orientation, glm::vec3{ 0, 1, 0 });

        // TODO : Update this comment to be world units/second
        // The primary velocity of the paritcles is 3 pixels/frame in the direction
        //  opposite to which the ship is travelling.
        auto baseVelocity = velocity * (-3.0f / glm::length(velocity)); // scale_to()
        // Calculate the sideways velocity for the two side streams.  The direction is
        //  perpendicular to the ship's velocity and the magnitude varies sinusoidally.
        auto perpendicularVelocity = glm::vec3(baseVelocity.z, baseVelocity.y, -baseVelocity.x) * (0.6f * std::sin(totalTime * 10.0f));

        // TODO : Documentation
        Particle particle{ };
        particle.type = Particle::Type::Enemy;
        particle.position = position; // +glm::vec3{ -25, 0, 0 } *particleRotation; // Position of the ship's exhaust pipe
        particle.duration = 60.0f;
        particle.scale = { 0.5f, 1.0f, 1.0f };
        auto& particleManager = Context::instance().particleManager;
        float alpha = 0.7f;

        // Middle particle stream
        auto middleColor = glm::vec4(1.0f, 187.0f / 255.0f, 30.0f / 255.0f, 1.0f); // Orange yellow
        particle.velocity = baseVelocity + get_random_vector(0, 1);
        particle.sprite = Sprite::Laser;
        particle.color = gvk::math::Color::White * alpha;
        particleManager.add(particle);
        particle.sprite = Sprite::Glow;
        particle.color = middleColor * alpha;
        particleManager.add(particle);

        // Side particle streams
        auto perpendicularVelocity0 = baseVelocity + perpendicularVelocity + get_random_vector(0.0f, 0.3f);
        auto perpendicularVelocity1 = baseVelocity - perpendicularVelocity + get_random_vector(0.0f, 0.3f);
        particle.sprite = Sprite::Laser;
        particle.color = gvk::math::Color::White * alpha;
        particle.velocity = perpendicularVelocity0;
        particleManager.add(particle);
        particle.velocity = perpendicularVelocity1;
        particleManager.add(particle);

        auto sideColor = glm::vec4(200.0f / 255.0f, 38.0f / 255.0f, 9.0f / 255.0f, 1.0f); // Deep red
        particle.sprite = Sprite::Glow;
        particle.color = sideColor * alpha;
        particle.velocity = perpendicularVelocity0;
        particleManager.add(particle);
        particle.velocity = perpendicularVelocity1;
        particleManager.add(particle);
    }
}

} // namespace shape_shooter
