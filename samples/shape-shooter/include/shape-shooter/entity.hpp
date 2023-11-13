
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

#include "shape-shooter/defines.hpp"
#include "shape-shooter/input-manager.hpp"

namespace shape_shooter {

class Entity
{
public:
    Entity(Sprite sprite);
    virtual ~Entity() = 0;

    glm::vec2 get_sprite_extent() const;
    virtual uint64_t get_type_id() const = 0;
    virtual void update(float deltaTime) = 0;
    virtual void draw(dst::gfx::SpriteRenderer& spriteRenderer) const;
    static bool collision(const Entity& lhs, const Entity& rhs);

    glm::vec3 position{ };
    glm::vec3 velocity{ };
    glm::vec4 color{ gvk::math::Color::White };
    float orientation{ };
    float radius{ 20 };
    bool expired{ };

protected:
    Sprite mSprite{ };

private:
    Entity(const Entity&) = delete;
    Entity& operator=(const Entity&) = delete;
};

} // namespace shape_shooter
