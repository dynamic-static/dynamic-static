
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

#include "dynamic-static/finite-state-machine.hpp"

#include "gtest/gtest.h"

class AttractState;
class PlayState;
class DeathState;
class GameOverState;

class AttractState final
    : public dst::State
{
public:

};

class PlayState final
    : public dst::State
{
public:

};

class DeathState final
    : public dst::State
{
public:

};

class GameOverState final
    : public dst::State
{
public:

};

TEST(FiniteStateMachine, Placeholder)
{
    dst::State::Machine gameState;
    gameState.register_state<AttractState>();
    gameState.register_state<PlayState>();
    gameState.register_state<DeathState>();
    gameState.register_state<GameOverState>();
    gameState.register_transition<AttractState, PlayState>();
    gameState.register_transition<PlayState, DeathState>();
    gameState.register_transition<DeathState, PlayState>();
    gameState.register_transition<DeathState, GameOverState>();
    gameState.register_transition<GameOverState, AttractState>();
}
