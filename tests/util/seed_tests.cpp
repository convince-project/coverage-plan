/**
 * Unit tests for the helper functions in seed.h/.cpp.
 * @see seed.h seed.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/util/seed.h"
#include <catch2/catch.hpp>
#include <cstring>
#include <random>

TEST_CASE("Tests for SeedHelpers::genRandomDeviceSeed",
          "[SeedHelpers::genRandomDeviceSeed]") {
  uint_fast64_t seed{SeedHelpers::genRandomDeviceSeed()};
  uint_fast64_t seedTwo{SeedHelpers::genRandomDeviceSeed()};

  REQUIRE(sizeof(seed) == 8);
  REQUIRE(seed != seedTwo);

  uint_fast64_t first32{seed >> 32};
  uint_fast64_t second32{(seed << 32) >> 32};
  REQUIRE(first32 != second32);

  first32 = seedTwo >> 32;
  second32 = (seedTwo << 32) >> 32;
  REQUIRE(first32 != second32);
}

TEST_CASE("Tests for SeedHelpers::doubleToUInt64",
          "[SeedHelpers::doubleToUInt64]") {
  double randNum = 1.5;
  uint_fast64_t randInt{SeedHelpers::doubleToUInt64(randNum)};
  uint_fast64_t randIntTwo{SeedHelpers::doubleToUInt64(randNum)};
  REQUIRE(randInt == randIntTwo);

  double randNumTwo = 1.5;
  uint_fast64_t randIntThree{SeedHelpers::doubleToUInt64(randNumTwo)};
  REQUIRE(randInt == randIntThree);

  double randNumThree = 2.7;
  uint_fast64_t randIntFour(SeedHelpers::doubleToUInt64(randNumThree));
  REQUIRE(randInt != randIntFour);

  double revertBack{};
  std::memcpy(&revertBack, &randInt, sizeof(double));
  REQUIRE(revertBack == 1.5);
}

TEST_CASE(
    "Tests for random seeding with mt19937_64 and uniform_real_distribution",
    "[mt19937_64 seedings]") {

  uint_fast64_t seed{SeedHelpers::genRandomDeviceSeed()};

  std::mt19937_64 gen{seed};

  std::uniform_real_distribution<double> sampler{0, 1};

  // Check we're getting a stream of random numbers
  double randOne{sampler(gen)};
  double randTwo{sampler(gen)};
  REQUIRE(randOne != randTwo);

  // generate some numbers to move through rng
  for (int i{0}; i < 10; ++i) {
    sampler(gen);
  }

  // Check if resetting seed half way through resets the distribution
  gen.seed(seed);
  double randThree{sampler(gen)};
  double randFour{sampler(gen)};
  REQUIRE(randOne == randThree);
  REQUIRE(randTwo == randFour);

  // Check if creating an entirely new instance creates copied numbers
  std::mt19937_64 genTwo{seed};
  std::uniform_real_distribution<double> samplerTwo{0, 1};
  double randFive{samplerTwo(genTwo)};
  double randSix{samplerTwo(genTwo)};
  REQUIRE(randOne == randFive);
  REQUIRE(randTwo == randSix);

  // Check setting something different gives stuff we don't want
  uint_fast64_t seedTwo{SeedHelpers::doubleToUInt64(73.76)};
  std::mt19937_64 genThree{seedTwo};
  std::uniform_real_distribution<double> samplerThree{0, 1};
  double randSeven{samplerThree(genThree)};
  double randEight{samplerThree(genThree)};
  REQUIRE(randOne != randSeven);
  REQUIRE(randTwo != randEight);
}