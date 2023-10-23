
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

#include "shape-shooter/input-manager.hpp"

namespace shape_shooter {

glm::vec3 InputManager::get_movement_direction() const
{
    glm::vec3 direction{ };
    // TODO : Thmubstick
    if (mInput.keyboard.down(gvk::system::Key::F)) { // Left
        direction.x += 1;
    }
    if (mInput.keyboard.down(gvk::system::Key::H)) { // Right
        direction.x -= 1;
    }
    if (mInput.keyboard.down(gvk::system::Key::T)) { // Forward
        direction.z -= 1;
    }
    if (mInput.keyboard.down(gvk::system::Key::G)) { // Backward
        direction.z += 1;
    }
    return direction.x || direction.y || direction.z ? glm::normalize(direction) : glm::vec3{ };
}

glm::vec3 InputManager::get_aim_direction() const
{
    return { };
}

void InputManager::update(const gvk::system::Input& input)
{
    mInput = input;
    const auto& mouseDelta = mInput.mouse.position.delta();
    if (
        mInput.keyboard.down(gvk::system::Key::UpArrow) ||
        mInput.keyboard.down(gvk::system::Key::DownArrow) ||
        mInput.keyboard.down(gvk::system::Key::LeftArrow) ||
        mInput.keyboard.down(gvk::system::Key::RightArrow) // TODO : Thumbsticks
    ) {
        mMouseAimEnabled = false;
    } else if (mouseDelta[0] || mouseDelta[1]) {
        mMouseAimEnabled = true;
    }
}

} // namespace shape_shooter
