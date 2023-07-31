/**
 * Utility functions for random seeding.
 *
 * @author Charlie Street
 *
 */

#ifndef SEED_H
#define SEED_H

#include <random>

namespace SeedHelpers {

/**
 * Generate a 64 bit seed for mt19937_64 using std::random_device.
 *
 * @return seed The 64 bit random seed
 */
uint_fast64_t genRandomDeviceSeed();

/**
 * Converts a 64 bit double to a uint_fast64_t, the seed type for mt19937_64.
 *
 * @param randNum The random double
 *
 * @return intSeed The seed converted from randNum
 */
uint_fast64_t doubleToUInt64(const double &randNum);

} // namespace SeedHelpers

#endif