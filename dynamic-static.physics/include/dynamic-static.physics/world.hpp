
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
#include "dynamic-static.physics/rigid-body.hpp"

#include <array>
#include <memory>
#include <set>

namespace dst {
namespace physics {

using Collision = std::array<const RigidBody*, 2>;

Collision make_collision(const RigidBody* pRigidBody0, const RigidBody* pRigidBody1);

class World final
{
public:
    struct CreateInfo final
    {
    };

    static void create(const CreateInfo* pCreateInfo, World* pWorld);

    ~World();

    const std::set<const RigidBody*>& get_collided_rigid_bodies() const;
    const std::set<Collision>& get_collisions() const;
    btVector3 get_gravity() const;
    void set_gravity(const btVector3& gravity);

    void make_dynamic(RigidBody& rigidBody);
    void make_static(RigidBody& rigidBody);
    void disable(RigidBody& rigidBody);
    void update(btScalar deltaTime);
    void clear();
    void reset();

private:
    static void bullet_physics_tick_callback(btDynamicsWorld* pDynamicsWorld, btScalar timeStep);

    std::unique_ptr<btCollisionConfiguration> mupCollisionConfiguration;
    std::unique_ptr<btCollisionDispatcher> mupDispatcher;
    std::unique_ptr<btBroadphaseInterface> mupBroadPhaseInterface;
    std::unique_ptr<btSequentialImpulseConstraintSolver> mupSolver;
    std::unique_ptr<btDiscreteDynamicsWorld> mupWorld;
    std::set<const RigidBody*> mCollidedRigidBodies;
    std::set<Collision> mCollisions;
};

} // namespace physics
} // namespace dst
