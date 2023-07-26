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
        IMacObservation{fov.at(i), (obsInt >> ((fovLength - 1) + i)) % 2 != 0});
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

  // TODO: Fill in
  return 0;
}