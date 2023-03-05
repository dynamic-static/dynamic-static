
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
#include <utility>

namespace dst {
namespace physics {

void RigidBody::create(const CreateInfo* pCreateInfo, RigidBody* pRigidBody)
{
    assert(pCreateInfo);
    assert(pCreateInfo->pCollisionShape);
    assert(pRigidBody);
    pRigidBody->reset();
    btVector3 localInertia { };
    pCreateInfo->pCollisionShape->calculateLocalInertia(pCreateInfo->mass, localInertia);
    pRigidBody->mupMotionState = std::make_unique<btDefaultMotionState>(pCreateInfo->initialTransform);
    pRigidBody->mupRigidBody = std::make_unique<btRigidBody>(pCreateInfo->mass, pRigidBody->mupMotionState.get(), pCreateInfo->pCollisionShape, localInertia);
    pRigidBody->mupRigidBody->setWorldTransform(pCreateInfo->initialTransform);
    pRigidBody->mupRigidBody->setCcdMotionThreshold((float)1e-7);
    pRigidBody->mupRigidBody->setDamping(pCreateInfo->linearDamping, pCreateInfo->angularDamping);
    pRigidBody->mupRigidBody->setLinearFactor(pCreateInfo->linearFactor);
    pRigidBody->mupRigidBody->setAngularFactor(pCreateInfo->angularFactor);
    pRigidBody->mupRigidBody->setFriction(pCreateInfo->material.friction);
    pRigidBody->mupRigidBody->setRestitution(pCreateInfo->material.restitution);
    pRigidBody->mupRigidBody->setUserPointer(pRigidBody);
    pRigidBody->mpUserData = pCreateInfo->pUserData;
}

RigidBody::RigidBody(RigidBody&& other) noexcept
{
    *this = std::move(other);
}

RigidBody& RigidBody::operator=(RigidBody&& other) noexcept
{
    if (this != &other) {
        mupMotionState = std::move(other.mupMotionState);
        mupRigidBody = std::move(other.mupRigidBody);
        mState = std::move(other.mState);
        mpUserData = std::move(other.mpUserData);
        mupRigidBody->setUserPointer(this);
    }
    return *this;
}

void RigidBody::reset()
{
    mupMotionState.reset();
    mupRigidBody.reset();
    mState = { };
    mpUserData = nullptr;
}

RigidBody::~RigidBody()
{
    reset();
}

RigidBody::State RigidBody::get_state() const
{
    return mState;
}

btTransform RigidBody::get_motion_state_transform() const
{
    assert(mupMotionState);
    assert(mupRigidBody);
    btTransform transform { };
    if (mState != State::Dynamic) {
        mupMotionState->setWorldTransform(mupRigidBody->getCenterOfMassTransform());
    }
    mupMotionState->getWorldTransform(transform);
    return transform;
}

const btTransform& RigidBody::get_transform() const
{
    assert(mupRigidBody);
    return mupRigidBody->getCenterOfMassTransform();
}

void RigidBody::set_transform(const btTransform& transform)
{
    assert(mupRigidBody);
    mupRigidBody->setCenterOfMassTransform(transform);
}

void* RigidBody::get_user_data() const
{
    return mpUserData;
}

void RigidBody::set_user_data(void* pUserData)
{
    mpUserData = pUserData;
}

void RigidBody::apply_impulse(const btVector3& impulse)
{
    assert(mupRigidBody);
    activate();
    mupRigidBody->applyCentralImpulse(impulse);
}

void RigidBody::apply_force(const btVector3& force)
{
    assert(mupRigidBody);
    activate();
    mupRigidBody->applyCentralForce(force);
}

void RigidBody::activate()
{
    assert(mupRigidBody);
    mupRigidBody->activate(true);
}

void RigidBody::halt()
{
    assert(mupRigidBody);
    mupRigidBody->setLinearVelocity({ 0, 0, 0 });
    mupRigidBody->setAngularVelocity({ 0, 0, 0 });
}

namespace detail {

const RigidBody* get_rigid_body(const btCollisionObject* pBtCollisionObject)
{
    assert(pBtCollisionObject);
    auto pRigidBody = (const RigidBody*)pBtCollisionObject->getUserPointer();
    assert(pRigidBody);
    return pRigidBody;
}

} // namespace detail
} // namespace physics
} // namespace dst
