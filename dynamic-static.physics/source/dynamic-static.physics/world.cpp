
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

#include "dynamic-static.physics/world.hpp"
#include "dynamic-static.physics/rigid-body.hpp"

#include <cassert>

namespace dst {
namespace physics {

void World::create(const CreateInfo* pCreateInfo, World* pWorld)
{
    assert(pCreateInfo);
    assert(pWorld);
    pWorld->mupCollisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
    pWorld->mupDispatcher = std::make_unique<btCollisionDispatcher>(pWorld->mupCollisionConfiguration.get());
    pWorld->mupBroadPhaseInterface = std::make_unique<btDbvtBroadphase>();
    pWorld->mupSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
    pWorld->mupWorld = std::make_unique<btDiscreteDynamicsWorld>(
        pWorld->mupDispatcher.get(),
        pWorld->mupBroadPhaseInterface.get(),
        pWorld->mupSolver.get(),
        pWorld->mupCollisionConfiguration.get()
    );
    pWorld->set_gravity(btVector3(0, -9.8f, 0));
}

btVector3 World::get_gravity() const
{
    assert(mupWorld);
    return mupWorld->getGravity();
}

void World::set_gravity(const btVector3& gravity)
{
    assert(mupWorld);
    mupWorld->setGravity(gravity);
}

void World::make_dynamic(RigidBody& rigidBody)
{
    assert(mupWorld);
    mupWorld->addRigidBody(rigidBody.mupRigidBody.get());
    rigidBody.mState = RigidBody::State::Dynamic;
    rigidBody.mupRigidBody->activate(true);
}

void World::make_static(RigidBody& rigidBody)
{
    assert(mupWorld);
    mupWorld->addCollisionObject(rigidBody.mupRigidBody.get());
    rigidBody.mState = RigidBody::State::Static;
}

void World::disable(RigidBody& rigidBody)
{
    assert(mupWorld);
#if 0
    rigidBody.mupRigidBody->clearForces();
    rigidBody.mupRigidBody->clearGravity();
    rigidBody.mupRigidBody->setLinearVelocity({ 0, 0, 0 });
    rigidBody.mupRigidBody->setAngularVelocity({ 0, 0, 0 });
#endif
    mupWorld->removeRigidBody(rigidBody.mupRigidBody.get());
    rigidBody.mState = RigidBody::State::Disabled;
}

void World::update(btScalar deltaTime)
{
    assert(mupWorld);
    mupWorld->stepSimulation(deltaTime);
}

} // namespace physics
} // namespace dst
