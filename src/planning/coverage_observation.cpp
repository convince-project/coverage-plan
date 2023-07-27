/**
 * Implementation of the observation helper functions in coverage_observation.h.
 * @see coverage_observation.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <cmath>
#include <despot/core/globals.h>
#include <tuple>
#include <vector>

/**
 * Converts a uint64_t representing an observation into a vector of
 * IMacObservations.
 */
std::pair<std::vector<IMacObservation>, bool>
Observation::fromObsType(const despot::OBS_TYPE &obsInt,
                         const std::vector<GridCell> &fov) {

  int fovLength{(int)fov.size()};
  if (fovLength > 63) {
    throw "FOV too big for uint64_t representation.";
  }

  std::vector<IMacObservation> obsVector{};

  // Get the leftmost bit in the integer
  bool actSuccess{(obsInt >> fovLength) == 1};

  // Shift so bit of interest is rightmost. Then do a modulo check for 0 or 1
  for (int i{0}; i < fovLength; ++i) {
    obsVector.push_back(
        IMacObservation{fov.at(i), (int)(obsInt >> ((fovLength - 1) - i)) % 2});
  }

  return std::make_pair(obsVector, actSuccess);
}

/**
 * Converts a vector of IMac observations and action success flag to a uint64_t
 * for use in DESPOT.
 */
despot::OBS_TYPE
Observation::toObsType(const std::vector<IMacObservation> &obsVector,
                       const ActionOutcome &outcome) {
  if (obsVector.size() > 63) {
    throw "FOV too big for uint64_t representation.";
  }

  int bitVal{(int)pow(2, obsVector.size())};

  // Convert bool to int
  despot::OBS_TYPE obsInt{(despot::OBS_TYPE)(((int)outcome.success) * bitVal)};

  // Continuously right shift the bitVal by 1 (i.e. divide by 2)
  bitVal = bitVal >> 1;

  // Assume same ordering as in obsVector
  for (const IMacObservation &imacObs : obsVector) {
    obsInt += imacObs.occupied * bitVal;
    bitVal = bitVal >> 1;
  }

  return obsInt;
}