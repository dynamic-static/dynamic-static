
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
#include "shape-shooter/context.hpp"
#include "shape-shooter/utilities.hpp"

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
        direction.z += 1;
    }
    if (mInput.keyboard.down(gvk::system::Key::G)) { // Backward
        direction.z -= 1;
    }
    return direction.x || direction.y || direction.z ? glm::normalize(direction) : glm::vec3{ };
}

glm::vec3 InputManager::get_aim_direction() const
{
    return mMouseAimEnabled ? get_mouse_aim_direction() : get_gamepad_aim_direction();
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

glm::vec3 InputManager::get_mouse_aim_direction() const
{
    const auto& context = Context::instance();
    const auto& renderExtent = context.renderExtent;
    glm::vec2 normalizedDeviceSpaceMouseRay{
        mInput.mouse.position.current[0] / renderExtent.x * 2 - 1,
        mInput.mouse.position.current[1] / renderExtent.y * 2 - 1,
    };
    glm::vec4 clipSpaceMouseRay{
        normalizedDeviceSpaceMouseRay.x,
        normalizedDeviceSpaceMouseRay.y,
        1.0f,
        1.0f
    };
    const auto& gameCamera = context.gameCamera;
    auto cameraSpaceMouseRay = glm::inverse(gameCamera.projection()) * clipSpaceMouseRay;
    cameraSpaceMouseRay.z = -1;
    cameraSpaceMouseRay.w = 0;
    auto worldSpaceMouseRay = glm::normalize(glm::inverse(gameCamera.view()) * cameraSpaceMouseRay);
    auto worldSpaceMouseRayOrigin = gameCamera.transform.translation;
    glm::vec3 worldSpaceMouseRayDirection{ worldSpaceMouseRay.x, worldSpaceMouseRay.y, worldSpaceMouseRay.z };
    auto aimPoint = ray_plane_intersection(worldSpaceMouseRayOrigin, worldSpaceMouseRayDirection, { }, { 0, 1, 0 });
    auto aimDirection = aimPoint - context.pPlayerShip->position;
    return aimDirection.x || aimDirection.y || aimDirection.z ? glm::normalize(aimDirection) : aimDirection;
}

glm::vec3 InputManager::get_gamepad_aim_direction() const
{
    return { };
}

} // namespace shape_shooter
