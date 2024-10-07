
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

#include "dynamic-static.audio/context.hpp"

#include "fmod.hpp"

#include <cassert>
#include <unordered_map>

#include <iostream>

#define fmod_result_scope

namespace dst {
namespace audio {

class Context::Impl final
{
public:
    FMOD::System* pFmodSystem{ nullptr };
    FMOD::ChannelGroup* pMusicChannelGroup{ nullptr };
    FMOD::ChannelGroup* pSoundEffectsChannelGroup{ nullptr };
    std::unordered_map<FMOD::Sound*, FMOD::Channel*> soundEffects;
};

Context::Context() = default;

bool Context::create(const CreateInfo& createInfo, Context* pContext)
{
    (void)createInfo;
    assert(pContext);
    auto upImpl = std::make_unique<Impl>();

    // Create FMOD::System
    auto result = FMOD::System_Create(&upImpl->pFmodSystem);
    assert(result == FMOD_OK);

    // Initialize FMOD
    result = upImpl->pFmodSystem->init(512, FMOD_INIT_NORMAL, nullptr);
    assert(result == FMOD_OK);

    // Create music FMOD::ChannelGroup
    result = upImpl->pFmodSystem->createChannelGroup("Music", &upImpl->pMusicChannelGroup);
    assert(result == FMOD_OK);

    // Create sound effects FMOD::ChannelGroup
    result = upImpl->pFmodSystem->createChannelGroup("Sound Effects", &upImpl->pSoundEffectsChannelGroup);
    assert(result == FMOD_OK);

    pContext->mupImpl = std::move(upImpl);
    return result == FMOD_OK;
}

Context::~Context() = default;

void Context::load_song()
{
    std::cout << "load_song()" << std::endl;
}

void Context::update()
{
    auto fmodResult = mupImpl->pFmodSystem->update();
    assert(fmodResult == FMOD_OK);
}

uint64_t Context::load_sound_effect(const char* pFilePath)
{
    assert(mupImpl);
    FMOD::Sound* pSound = nullptr;
    auto result = mupImpl->pFmodSystem->createSound(pFilePath, FMOD_DEFAULT, nullptr, &pSound);
    assert(result == FMOD_OK);
    auto inserted = mupImpl->soundEffects.insert({ pSound, nullptr }).second;
    assert(inserted);
    return (uint64_t)pSound;
}

void Context::play_sound_effect(uint64_t soundEffectId) const
{
    auto fmodResult = mupImpl->pFmodSystem->playSound((FMOD::Sound*)soundEffectId, nullptr, false, nullptr);
    assert(fmodResult == FMOD_OK);
}

} // namespace audio
} // namespace dst
