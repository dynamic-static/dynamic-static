
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

namespace shape_shooter {

class Particle final
{
public:
    enum class Type
    {
        None,
        Enemy,
        Bullet,
        IgnoreGravity,
    };

    Type type{ };
    Sprite sprite{ Sprite::Laser };
    glm::vec3 position{ };
    glm::vec3 velocity{ };
    float orientation{ };
    glm::vec3 scale{ 1, 1, 1 };
    glm::vec4 color{ gvk::math::Color::White };
    float duration{ };
    float percentLife{ 1 };
    float lengthMultiplier{ 1 };
};

} // namespace shape_shooter
