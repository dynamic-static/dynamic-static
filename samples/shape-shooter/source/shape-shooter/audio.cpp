
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

#include "shape-shooter/audio.hpp"
#include "shape-shooter/context.hpp"

namespace shape_shooter {

bool Audio::create(Audio* pAudio)
{
    assert(pAudio);
    dst::audio::Context::CreateInfo audioContextCreateInfo{ };
    auto success = dst::audio::Context::create(audioContextCreateInfo, &pAudio->mContext);
    for (const auto& directoryEntry : std::filesystem::directory_iterator(SHAPE_SHOOTER_CONTENT "/Audio/")) {
        if (gvk::string::contains(directoryEntry.path().filename().string(), "explosion")) {
            pAudio->mExplosionSounedEffects.push_back(pAudio->mContext.load_sound_effect(directoryEntry.path().string().c_str()));
        } else
        if (gvk::string::contains(directoryEntry.path().filename().string(), "shoot")) {
            pAudio->mShotSoundEffects.push_back(pAudio->mContext.load_sound_effect(directoryEntry.path().string().c_str()));
        } else
        if (gvk::string::contains(directoryEntry.path().filename().string(), "spawn")) {
            pAudio->mSpawnSoundEffects.push_back(pAudio->mContext.load_sound_effect(directoryEntry.path().string().c_str()));
        }
    }
    return success;
}

void Audio::play(SoundEffect soundEffect)
{
    auto& rng = get_context().rng;
    switch (soundEffect) {
    case SoundEffect::Explosion: {
        mContext.play_sound_effect(mExplosionSounedEffects[rng.index(mExplosionSounedEffects.size())]);
    } break;
    case SoundEffect::Shot: {
        mContext.play_sound_effect(mShotSoundEffects[rng.index(mShotSoundEffects.size())]);
    } break;
    case SoundEffect::Spawn: {
        mContext.play_sound_effect(mSpawnSoundEffects[rng.index(mSpawnSoundEffects.size())]);
    } break;
    default: {
        assert(false);
    } break;
    }
}

void Audio::update()
{
    mContext.update();
}

} // namespace shape_shooter
