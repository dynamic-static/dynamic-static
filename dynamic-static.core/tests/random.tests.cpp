
/*
==========================================
  Copyright (c) 2016-2021 dynamic_static
      Licensed under the MIT license
    http://opensource.org/licenses/MIT
==========================================
*/

#if 0

#include "dynamic_static/random.hpp"

#include "catch2/catch.hpp"

#include <vector>

namespace dst {
namespace tests {

static constexpr int MaxValue { 4 };
static constexpr int MinValue { -MaxValue };
static constexpr int TestCount { 1024 };

/**
Validates that RandomNumberGenerator::range() stays in range
*/
TEST_CASE("RandomNumberGenerator::range()", "[RandomNumberGenerator]")
{
    RandomNumberGenerator rng;
    SECTION("int stays in range")
    {
        for (int i = 0; i < TestCount; ++i) {
            auto value = rng.range(MinValue, MaxValue);
            if (value < MinValue || MaxValue < value) {
                FAIL();
            }
        }
    }
    SECTION("float stays in range")
    {
        for (int i = 0; i < TestCount; ++i) {
            auto value = rng.range((float)MinValue * 0.1f, (float)MaxValue * 0.1f);
            if (value < MinValue || MaxValue < value) {
                FAIL();
            }
        }
    }
}

/**
Validates that RandomNumberGenerator::probability() always passes/fails at upper/lower bound
*/
TEST_CASE("RandomNumberGenerator::probability()", "[RandomNumberGenerator]")
{
    RandomNumberGenerator rng;
    SECTION("int 0 always fails")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (rng.probability(0)) {
                FAIL();
            }
        }
    }
    SECTION("int 100 always passes")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (!rng.probability(100)) {
                FAIL();
            }
        }
    }
    SECTION("float 0.0f always fails")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (rng.probability(0.0f)) {
                FAIL();
            }
        }
    }
    SECTION("float 1.0f always passes")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (!rng.probability(1.0f)) {
                FAIL();
            }
        }
    }
}

/**
Validates that RandomNumberGenerator::index() stays in range
*/
TEST_CASE("RandomNumberGenerator::index()", "[RandomNumberGenerator]")
{
    RandomNumberGenerator rng;
    SECTION("count 0 always gets index 0")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (rng.index(0) != 0) {
                FAIL();
            }
        }
    }
    SECTION("count 1 always gets index 0")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (rng.index(1) != 0) {
                FAIL();
            }
        }
    }
    SECTION("count 8 stays in range")
    {
        for (int i = 0; i < TestCount; ++i) {
            auto index = rng.index(8);
            if (index < 0 || 7 < index) {
                FAIL();
            }
        }
    }
}

/**
Validates that RandomNumberGenerator::die_roll() stays in range
*/
TEST_CASE("RandomNumberGenerator::die_roll()", "[RandomNumberGenerator]")
{
    RandomNumberGenerator rng;
    SECTION("D0 always rolls 0")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (rng.die_roll(0) != 0) {
                FAIL();
            }
        }
    }
    SECTION("D1 always rolls 1")
    {
        for (int i = 0; i < TestCount; ++i) {
            if (rng.die_roll(1) != 1) {
                FAIL();
            }
        }
    }
    SECTION("D6 stays in range")
    {
        for (int i = 0; i < TestCount; ++i) {
            auto diRoll = rng.die_roll(6);
            if (diRoll < 0 || 6 < diRoll) {
                FAIL();
            }
        }
    }
}

/**
Validates that RandomNumberGenerator::reset() produces deterministic sequences
*/
TEST_CASE("RandomNumberGenerator::reset()", "[RandomNumberGenerator]")
{
    RandomNumberGenerator rng;
    std::vector<int> ints(TestCount);
    std::vector<float> floats(TestCount);
    for (size_t i = 0; i < TestCount; ++i) {
        ints[i] = rng.value<int>();
        floats[i] = rng.value<float>();
    }
    rng.reset();
    for (size_t i = 0; i < TestCount; ++i) {
        if (rng.value<int>() != ints[i] || rng.value<float>() != floats[i]) {
            FAIL();
        }
    }
}

} // namespace tests
} // namespace dst

#endif
