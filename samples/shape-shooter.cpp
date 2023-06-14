
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

#include "dynamic-static.sample-utilities.hpp"

#include <filesystem>

int main(int, const char* [])
{
    FMOD::System* pFmodSystem = nullptr;
    auto fmodResult = FMOD::System_Create(&pFmodSystem);
    assert(fmodResult == FMOD_OK);

    int audioDriverCount = 0;
    fmodResult = pFmodSystem->getNumDrivers(&audioDriverCount);
    assert(fmodResult == FMOD_OK);
    assert(audioDriverCount);

    fmodResult = pFmodSystem->init(36, FMOD_INIT_NORMAL, NULL);
    assert(fmodResult == FMOD_OK);

    FMOD::Sound* pFmodSound = nullptr;
    fmodResult = pFmodSystem->createSound(SHAPE_SHOOTER_CONTENT "/Audio/Music.mp3", FMOD_DEFAULT, 0, &pFmodSound);
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSound->setMode(FMOD_LOOP_NORMAL);
    assert(fmodResult == FMOD_OK);
    fmodResult = pFmodSound->setLoopCount(-1);
    assert(fmodResult == FMOD_OK);
    fmodResult = pFmodSystem->playSound(pFmodSound, nullptr, false, nullptr);
    assert(fmodResult == FMOD_OK);

    bool loop = true;
    while (loop) {
        int b = 0;
        (void)b;
    }

    fmodResult = pFmodSound->release();
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSystem->close();
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSystem->release();
    assert(fmodResult == FMOD_OK);

    return 0;
}
