
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

#include "shape-shooter/defines.hpp"
#include "shape-shooter/enemy-spawner.hpp"
#include "shape-shooter/entity-manager.hpp"
#include "shape-shooter/grid.hpp"
#include "shape-shooter/input-manager.hpp"
#include "shape-shooter/particle-manager.hpp"
#include "shape-shooter/player-ship.hpp"
#include "shape-shooter/player-status.hpp"
#include "shape-shooter/score-board.hpp"

namespace shape_shooter {

class Context final
{
public:
    static Context& instance();
    glm::vec2 renderExtent{ };
    dst::RandomNumberGenerator rng;
    dst::gfx::SpriteRenderer spriteRenderer;
    PlayField playField{ { 1920, 1, 1080 } };
    PlayerStatus playerStatus;
    ScoreBoard scoreBoard;
    gvk::math::Camera gameCamera;
    PlayerShip* pPlayerShip{ };
    EnemySpawner enemySpawner;
    EntityManager entityManager;
    InputManager inputManager;
    ParticleManager particleManager;
    Grid grid;

private:
    Context();

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
};

} // namespace shape_shooter
