
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
#include "shape-shooter/entity.hpp"
#include "shape-shooter/input-manager.hpp"

namespace shape_shooter {

class PlayerShip
    : public Entity
{
public:
    static constexpr uint32_t CooldownFrames{ 6 };

    PlayerShip();

    uint64_t get_type_id() const override final;
    bool is_dead() const;
    void update(float deltaTime) override final;
    void draw(dst::gfx::SpriteRenderer& spriteRenderer) const override final;

private:
    float mSpeed{ 8.0f / (1.0f / 60.0f) };
    float mCooldownTime{ 0.1f };
    float mCooldownTimer{ };

};

} // namespace shape_shooter
