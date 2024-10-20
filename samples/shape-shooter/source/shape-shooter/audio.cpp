
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

#define DST_FMOD_ENABLED 0

namespace shape_shooter {

bool Audio::create(Audio* pAudio)
{
#if DST_FMOD_ENABLED
    assert(pAudio);
    dst::audio::Context::CreateInfo audioContextCreateInfo{ };
    auto success = dst::audio::Context::create(audioContextCreateInfo, &pAudio->mContext);
    pAudio->mSong = pAudio->mContext.load_song(SHAPE_SHOOTER_CONTENT "/Audio/Music.mp3");
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
    pAudio->mContext.play_song(pAudio->mSong);
    return success;
#else
    (void)pAudio;
    return true;
#endif
}

void Audio::play(SoundEffect soundEffect)
{
#if DST_FMOD_ENABLED
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
#else
    (void)soundEffect;
#endif
}

void Audio::update()
{
#if DST_FMOD_ENABLED
    mContext.update();
#endif
}

} // namespace shape_shooter
