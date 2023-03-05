
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

#include <algorithm>
#include <cassert>

namespace dst {
namespace physics {

Collision make_collision(const RigidBody* pRigidBody0, const RigidBody* pRigidBody1)
{
    return pRigidBody0 < pRigidBody1 ? Collision { pRigidBody0, pRigidBody1 } : Collision { pRigidBody1, pRigidBody0 };
}

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
    pWorld->mupWorld->setWorldUserInfo(pWorld);
    pWorld->mupWorld->setInternalTickCallback(bullet_physics_tick_callback, pWorld);
}

World::~World()
{
    reset();
}

const std::set<const RigidBody*>& World::get_collided_rigid_bodies() const
{
    return mCollidedRigidBodies;
}

const std::set<Collision>& World::get_collisions() const
{
    return mCollisions;
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
    mupWorld->removeRigidBody(rigidBody.mupRigidBody.get());
    rigidBody.mState = RigidBody::State::Disabled;
}

void World::update(btScalar deltaTime)
{
    assert(mupWorld);
    mCollidedRigidBodies.clear();
    mCollisions.clear();
    mupWorld->stepSimulation(deltaTime);
}

void World::clear()
{
    if (mupWorld) {
        mCollidedRigidBodies.clear();
        mCollisions.clear();
        for (int i = mupWorld->getNumCollisionObjects() - 1; 0 <= i; --i) {
            auto pCollisionObject = mupWorld->getCollisionObjectArray()[i];
            mupWorld->removeCollisionObject(pCollisionObject);
        }
    }
}

void World::reset()
{
    clear();
    mupCollisionConfiguration.reset();
    mupDispatcher.reset();
    mupBroadPhaseInterface.reset();
    mupSolver.reset();
    mupWorld.reset();
    mCollidedRigidBodies.clear();
    mCollisions.clear();
}

void World::bullet_physics_tick_callback(btDynamicsWorld* pDynamicsWorld, btScalar)
{
    auto pWorld = (World*)pDynamicsWorld->getWorldUserInfo();
    auto& collisions = pWorld->mCollisions;
    auto& collidedRigidBodies = pWorld->mCollidedRigidBodies;
    auto pDispatcher = pDynamicsWorld->getDispatcher();
    auto numManifolds = pDispatcher->getNumManifolds();
    for (int manifold_i = 0; manifold_i < numManifolds; ++manifold_i) {
        auto pManifold = pDispatcher->getManifoldByIndexInternal(manifold_i);
        for (int contact_i = 0; contact_i < pManifold->getNumContacts(); ++contact_i) {
            if (pManifold->getContactPoint(contact_i).getDistance() < 0) {
                auto collision = make_collision(detail::get_rigid_body(pManifold->getBody0()), detail::get_rigid_body(pManifold->getBody1()));
                collidedRigidBodies.insert(collision[0]);
                collidedRigidBodies.insert(collision[1]);
                collisions.insert(collision);
            }
        }
    }
}

} // namespace physics
} // namespace dst
