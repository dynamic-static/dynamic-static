
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

#include "../../dynamic-static.sample-utilities.hpp"

#include "dynamic-static/text.hpp"
#include "dynamic-static.graphics/coordinate-renderer.hpp"
#include "dynamic-static.graphics/text.hpp"
#include "dynamic-static.graphics/line-renderer.hpp"
#include "dynamic-static.graphics/sprite-renderer.hpp"

#include <array>

namespace shape_shooter {

class Context;

enum class Sprite
{
    BlackHole,
    Bullet,
    Glow,
    Laser,
    PlayerShip,
    Pointer,
    Seeker,
    Wanderer,
    Count,
};

inline constexpr std::array<const char*, (uint32_t)Sprite::Count> SpriteFilePaths {
    SHAPE_SHOOTER_CONTENT "/Art/Black Hole.png",
    SHAPE_SHOOTER_CONTENT "/Art/Bullet.png",
    SHAPE_SHOOTER_CONTENT "/Art/Glow.png",
    SHAPE_SHOOTER_CONTENT "/Art/Laser.png",
    SHAPE_SHOOTER_CONTENT "/Art/Player.png",
    SHAPE_SHOOTER_CONTENT "/Art/Pointer.png",
    SHAPE_SHOOTER_CONTENT "/Art/Seeker.png",
    SHAPE_SHOOTER_CONTENT "/Art/Wanderer.png",
};

// inline constexpr float SpriteScale{ 0.1f };
inline constexpr float SpriteScale{ 1.0f };
inline constexpr glm::vec3 SpriteOffset{ 0, 1, 0 };
inline constexpr float OneOverSixty{ 0.0166666666666667f };

} // namespace shape_shooter
