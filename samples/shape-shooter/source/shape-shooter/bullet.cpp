
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

void Bullet::update(float deltaTime)
{
    (void)deltaTime;
    position += velocity;
    // Context::instance().grid.apply_explosive_force(0.5f * glm::length(velocity), position, /* from_1920x1080(64, 80) */ 2.666666666666669f);
    Context::instance().grid.apply_explosive_force(0.5f * glm::length(velocity), position, 80.0f);
    if (position.x < -1920 || 1920 < position.x ||
        position.z < -1080 || 1080 < position.z) {
        expired = true;
        for (uint32_t i = 0; i < 30; ++i) {
            (void)i;
            // TODO : Create particles
        }
    }
}

} // namespace shape_shooter
