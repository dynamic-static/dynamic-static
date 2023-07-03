
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

#include "dynamic-static/defines.hpp"
#include "dynamic-static/type-id.hpp"

#include <cassert>
#include <functional>
#include <memory>
#include <unordered_map>

namespace dst {

template <typename ...UpdateArgsTypes>
class State
{
public:
    class Machine;

    State() = default;
    State(State&&) = default;
    State& operator=(State&&) = default;

    inline virtual ~State() = 0
    {
    }

    inline virtual void enter(const State* pExiting)
    {
        (void)pExiting;
    }

    inline virtual void update(State::Machine& stateMachine, UpdateArgsTypes&&...)
    {
        (void)stateMachine;
    }

    inline virtual void exit(const State* pEntering)
    {
        (void)pEntering;
    }

    class Machine final
    {
    public:
        Machine() = default;
        Machine(Machine&&) = default;
        Machine& operator=(Machine&&) = default;

        template <typename StateType, typename ...CtorArgsTypes>
        inline void add_state(CtorArgsTypes&&... ctorArgs)
        {
            auto inserted = mStates.insert({ get_type_id<StateType>(), std::make_unique<StateType>(std::forward<CtorArgsTypes>(ctorArgs)...) }).second;
            (void)inserted;
            assert(inserted);
        }

        template <typename StateType = void>
        inline void set_state()
        {
            auto pExiting = mpState;
            auto itr = mStates.find(get_type_id<StateType>());
            auto pEntering = itr != mStates.end() ? itr->second.get() : nullptr;

            if (pExiting) {
                pExiting->exit(pEntering);
            }

            mpState = pEntering;

            if (pEntering) {
                pEntering->enter(pExiting);
            }
        }

        inline void update(UpdateArgsTypes&&... updateArgs)
        {
            if (mpState) {
                mpState->update(*this, std::forward<UpdateArgsTypes>(updateArgs)...);
            }
        }

    private:
        std::unordered_map<TypeId, std::unique_ptr<State>> mStates;
        State* mpState { };

        Machine(const Machine&) = delete;
        Machine& operator=(const Machine&) = delete;
    };

private:
    State(const State&) = delete;
    State& operator=(const State&) = delete;
};

} // namespace dst
