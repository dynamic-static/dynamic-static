
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

#include "shape-shooter/bullet.hpp"
#include "shape-shooter/context.hpp"
#include "shape-shooter/utilities.hpp"

namespace shape_shooter {

Bullet::Bullet(const glm::vec3& pos, const glm::vec3& vel)
    : Entity(Sprite::Bullet)
{
    position = pos;
    velocity = vel;
    orientation = get_orientation(vel);
    radius = 8;
}

uint64_t Bullet::get_type_id() const
{
    return shape_shooter::get_type_id<Bullet>();
}

void Bullet::update()
{
    auto& context = Context::instance();
    const auto& playField = context.playField;
    auto& particleManager = context.particleManager;

    // position += velocity; // deltaTime
    // Context::instance().grid.apply_explosive_force(0.5f * glm::length(velocity), position, /* from_1920x1080(64, 80) */ 2.666666666666669f);
    Context::instance().grid.apply_explosive_force(0.5f * glm::length(velocity), position, 80.0f);
    if (!playField.contains(position)) {
        expired = true;
        for (uint32_t i = 0; i < 30; ++i) {
            Particle particle{ };
            particle.position = position;
            particle.velocity = get_random_vector(0, 9);
            particle.color = gvk::math::Color::LightBlue;
            particle.duration = 50;
            particle.scale = { 1, 1, 1 };
            particle.type = Particle::Type::Bullet;
            particleManager.add(particle);
        }
    }
}

} // namespace shape_shooter
