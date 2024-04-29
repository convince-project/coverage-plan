/**
 * @file coverage_observation.h
 *
 * @brief Conversions between readable observations and DESPOT representations.
 *
 * I would represent observations as a vector of IMacObservations.
 * DESPOT represents all objects as uint64_t (aka OBS_TYPE).
 *
 * @author Charlie Street
 */

#ifndef COVERAGE_OBSERVATION_H
#define COVERAGE_OBSERVATION_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <despot/core/globals.h>
#include <tuple>
#include <vector>

namespace Observation {

/**
 * Converts a uint64_t representing an observation into a vector of
 * IMacObservations.
 *
 * This is much easier for me to work with.
 *
 * @param obsInt The uint64_t representing the observation. The first bit is an
 * action success flag. The remaining bits are the map observations in fov
 * order
 * @param fov A vector of GridCells releative to the robot's position capturing
 * its field of view
 * @param robotPos The robot's current position, allows the output vector to
 * have absolute positions (optional, we may want the relative observations)
 *
 * @returns A vector of observations capturing the robot's absolute
 * observation paired with an action success flag
 *
 * @exception tooManyCells Raised if > 63 cells in FOV
 */
std::pair<std::vector<IMacObservation>, bool>
fromObsType(const despot::OBS_TYPE &obsInt, const std::vector<GridCell> &fov,
            const GridCell &robotPos = GridCell{0, 0});

/**
 * Converts a vector of IMac observations and action success flag to a uint64_t
 * for use in DESPOT.
 *
 * @param obsVector A vector of IMacObservations in FOV order (can be relative
 * or absolute)
 * @param outcome An ActionOutcome struct capturing the robot's action success
 *
 * @returns The observation as an integer
 *
 * @exception tooManyCells Raised if > 63 cells in FOV
 */
despot::OBS_TYPE toObsType(const std::vector<IMacObservation> &obsVector,
                           const ActionOutcome &outcome);

/**
 * Compute the observation given the map, robot position and fov.
 *
 * @param map The Eigen matrix capturing the state of the environment
 * @param robotPos The robot's position
 * @param outcome The action outcome
 * @param fov The robot's field of view as a vector of relative grid cells
 *
 * @returns The observation as a number
 */
despot::OBS_TYPE computeObservation(const Eigen::MatrixXi &map,
                                    const GridCell &robotPos,
                                    const ActionOutcome &outcome,
                                    const std::vector<GridCell> &fov);

} // namespace Observation
#endif