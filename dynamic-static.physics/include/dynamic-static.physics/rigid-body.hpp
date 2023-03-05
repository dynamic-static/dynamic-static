
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

#include "dynamic-static.physics/defines.hpp"
#include "dynamic-static.physics/material.hpp"

#include <memory>

namespace dst {
namespace physics {

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

    RigidBody() = default;
    static void create(const CreateInfo* pCreateInfo, RigidBody* pRigidBody);
    RigidBody(RigidBody&& other) noexcept;
    RigidBody& operator=(RigidBody&& other) noexcept;
    void reset();
    ~RigidBody();

    State get_state() const;
    const btTransform& get_transform() const;
    void set_transform(const btTransform& transform);
    void* get_user_data() const;
    void set_user_data(void* pUserData);

    void apply_impulse(const btVector3& impulse);
    void apply_force(const btVector3& force);
    void activate();
    void halt();

public:
    std::unique_ptr<btMotionState> mupMotionState;
    std::unique_ptr<btRigidBody> mupRigidBody;
    State mState { State::Disabled };
    void* mpUserData { nullptr };
    friend class World;
};

namespace detail {

const RigidBody* get_rigid_body(const btCollisionObject* pBtCollisionObject);

} // namespace detail
} // namespace physics
} // namespace dst
