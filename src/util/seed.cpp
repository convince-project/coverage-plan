/**
 * Implementation of functions in seed.h.
 * @see seed.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/util/seed.h"
#include <cstring>
#include <random>

/**
 * Generate a 64 bit seed for mt19937_64 using std::random_device.
 */
uint_fast64_t SeedHelpers::genRandomDeviceSeed() {
  static_assert(sizeof(uint_fast64_t) == 8, "uint_fast64_t not 8 bytes");
  static_assert(sizeof(std::random_device::result_type) == 4,
                "Random device result type not 4 bytes");
  std::random_device rnd{};
  return (static_cast<uint_fast64_t>(rnd()) << 32) | rnd();
}

/**
 * Converts a 64 bit double to a uint_fast64_t, the seed type for mt19937_64.
 */
uint_fast64_t SeedHelpers::doubleToUInt64(const double &randNum) {
  static_assert(sizeof(uint_fast64_t) == 8, "uint_fast64_t not 8 bytes");
  static_assert(sizeof(double) == 8, "Double not 8 bytes");
  uint_fast64_t seed{};
  std::memcpy(&seed, &randNum, sizeof(seed));
  return seed;
}