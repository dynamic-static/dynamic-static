
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
{
    imageIndex = (uint32_t)shape_shooter::Sprite::Bullet;
    const auto& images = Context::instance().spriteRenderer.get_images();
    const auto& imageCreateInfo = images[imageIndex].get<gvk::Image>().get<VkImageCreateInfo>();
    extent = glm::vec2{ imageCreateInfo.extent.width, imageCreateInfo.extent.height };
    position = pos;
    velocity = vel;
    orientation = get_orientation(vel);
    radius = 8 * SpriteScale;
}

void Bullet::update(const InputManager& inputManager, float deltaTime)
{
    (void)inputManager;
    (void)deltaTime;
    position += velocity;
    // grid.apply_explosive_force();
    if (position.x < -32 || 32 < position.x ||
        position.y < -32 || 32 < position.y) {
        expired = true;
        for (uint32_t i = 0; i < 30; ++i) {
            (void)i;
            // TODO : Create particles
        }
    }
}

} // namespace shape_shooter
