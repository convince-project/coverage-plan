/**
 * @file random_coverage_robot.h
 *
 * @brief Header file for the RandomCoverageRobot class.
 *
 * This is an implementation of CoverageRobot which takes random actions.
 *
 * @author Charlie Street
 */
#ifndef RANDOM_COVERAGE_ROBOT_H
#define RANDOM_COVERAGE_ROBOT_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/coverage_robot.h"
#include "coverage_plan/planning/coverage_world.h"
#include <vector>

/**
 * A class for a random coverage robot, i.e. the robot moves randomly.
 *
 * Members:
 * As in superclass, plus:
 * * _world: The CoverageWorld representing the environment
 * * _fov: The robot's FOV represented as a vector of relative grid cells
 */
class RandomCoverageRobot : public CoverageRobot {

private:
  std::shared_ptr<CoverageWorld> _world{};
  const std::vector<GridCell> _fov{};

  /**
   * Synthesises a random valid action for the coverage robot.
   * Recall that x goes from left to right, y from top to bottom.
   *
   * @param currentLoc The robot's current location
   * @param enabledActions A vector of enabled actions in this state
   * @param ts The current timestep
   * @param timeBound The time bound
   * @param imac The current IMac instance
   * @param visited The vector of visited locations
   * @param currentObs The most recent observations
   *
   * @returns The next action to be executed
   */
  Action _planFn(const GridCell &currentLoc,
                 const std::vector<Action> &enabledActions, int ts,
                 int timeBound, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &visited,
                 const std::vector<IMacObservation> &currentObs);

  /**
   * Executes an action by checking against the CoverageWorld object.
   * Just applies the action to the grid and checks the outcome.
   *
   * @param currentLoc The robot's current location
   * @param action The action to execute
   *
   * @returns The outcome of the action
   */
  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action);

  /**
   *  Dummy observation function which returns an empty vector.
   *
   * @param currentLoc The robot's current location
   *
   * @returns A vector of observations
   */
  std::vector<IMacObservation> _observeFn(const GridCell &currentLoc);

public:
  /**
   * Constructor calls super constructor and initialises _world.
   *
   * @param currentLoc The robot's current location
   * @param timeBound The planning time bound
   * @param xDim The x dimension of the map
   * @param yDim The y dimension of the map
   * @param world The IMacExecutor representing the environment
   * @param fov The robot's FOV as a vector of relative grid cells
   * @param groundTruthIMac The ground truth IMac instance (if we don't want to
   * use BiMac)
   * @param estimationType The type of parameter estimation to use for IMac
   * instance for episode
   */
  RandomCoverageRobot(const GridCell &currentLoc, int timeBound, int xDim,
                      int yDim, std::shared_ptr<CoverageWorld> world,
                      const std::vector<GridCell> &fov,
                      std::shared_ptr<IMac> groundTruthIMac = nullptr,
                      const ParameterEstimate &estimationType =
                          ParameterEstimate::posteriorSample)
      : CoverageRobot{currentLoc, timeBound,       xDim,
                      yDim,       groundTruthIMac, estimationType},
        _world{world}, _fov{fov} {}

  /**
   * Runs necessary setup for an episode, like resetting the CoverageWorld.
   *
   * @param startLoc The robot's initial location for the episode
   * @param ts The initial timestep
   * @param timeBound The episode time bound, which could change
   * @param imacForEpisode The IMac instance being used for the planning episode
   */
  void episodeSetup(const GridCell &startLoc, const int &ts,
                    const int &timeBound, std::shared_ptr<IMac> imacForEpisode);
};

#endif