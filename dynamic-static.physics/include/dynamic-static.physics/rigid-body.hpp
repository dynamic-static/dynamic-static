
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

#include "dynamic-static.physics/collider.hpp"
#include "dynamic-static.physics/defines.hpp"
#include "dynamic-static.physics/material.hpp"

#include <memory>

namespace dst {
namespace physics {

class World;

class RigidBody final
{
public:
    enum class State
    {
        Disabled = 0,
        Dynamic,
        Static,
    };

    struct CreateInfo final
    {
        btScalar mass { 0 };
        Material material { };
        btScalar linearDamping { 0 };
        btScalar angularDamping { 0 };
        btVector3 linearFactor { 1, 1, 1 };
        btVector3 angularFactor { 1, 1, 1 };
        btTransform initialTransform { btTransform::getIdentity() };
        btCollisionShape* pCollisionShape { nullptr };
        void* pUserData { nullptr };
    };

    static void create(const CreateInfo* pCreateInfo, RigidBody* pRigidBody);

    State get_state() const;

public:
    std::unique_ptr<btMotionState> mupMotionState;
    std::unique_ptr<btRigidBody> mupRigidBody;
    State mState { State::Disabled };
    friend class World;
};

} // namespace physics
} // namespace dst
