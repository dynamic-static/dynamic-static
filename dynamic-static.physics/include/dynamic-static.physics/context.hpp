
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

#include "dynamic-static.physics/defines.hpp"

#include <cassert>
#include <memory>

namespace dst {
namespace physics {

class Context final
{
public:
    inline static void create(Context* pContext)
    {
        assert(pContext);
        pContext->reset();
        pContext->mupCollisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
        pContext->mupDispatcher = std::make_unique<btCollisionDispatcher>(pContext->mupCollisionConfiguration.get());
        pContext->mupBroadPhaseInterface = std::make_unique<btDbvtBroadphase>();
        pContext->mupSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
        pContext->mupWorld = std::make_unique<btDiscreteDynamicsWorld>(
            pContext->mupDispatcher.get(),
            pContext->mupBroadPhaseInterface.get(),
            pContext->mupSolver.get(),
            pContext->mupCollisionConfiguration.get()
        );
        pContext->mupWorld->setGravity(btVector3(0, -9.8f, 0));
    }

    inline btDiscreteDynamicsWorld* get_dynamics_world()
    {
        return mupWorld.get();
    }

    inline void reset()
    {
        mupWorld.reset();
        mupSolver.reset();
        mupBroadPhaseInterface.reset();
        mupDispatcher.reset();
        mupCollisionConfiguration.reset();
    }

public:
    std::unique_ptr<btCollisionConfiguration> mupCollisionConfiguration;
    std::unique_ptr<btCollisionDispatcher> mupDispatcher;
    std::unique_ptr<btBroadphaseInterface> mupBroadPhaseInterface;
    std::unique_ptr<btSequentialImpulseConstraintSolver> mupSolver;
    std::unique_ptr<btDiscreteDynamicsWorld> mupWorld;
};

} // namespace physics
} // namespace dst
