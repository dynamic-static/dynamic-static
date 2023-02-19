
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

#include "dynamic-static.physics/rigid-body.hpp"

#include <cassert>

namespace dst {
namespace physics {

bool RigidBody::create(const CreateInfo* pCreateInfo, RigidBody* pRigidBody)
{
    assert(pCreateInfo);
    assert(pCreateInfo->pCollisionShape);
    assert(pRigidBody);
    pRigidBody->mupMotionState = std::make_unique<btDefaultMotionState>();
    pRigidBody->mupRigidBody = std::make_unique<btRigidBody>(pCreateInfo->mass, pRigidBody->mupMotionState.get(), pCreateInfo->pCollisionShape);
    pRigidBody->mupRigidBody->setCcdMotionThreshold((float)1e-7);
    return true;
}

btTransform RigidBody::get_transform() const
{
    assert(mupMotionState);
    btTransform transform { };
    mupMotionState->getWorldTransform(transform);
    return transform;
}

void RigidBody::set_transform(const btTransform& transform)
{
    assert(mupMotionState);
    mupMotionState->setWorldTransform(transform);
}

} // namespace physics
} // namespace dst