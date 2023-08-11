/**
 * Header file for the RandomCoverageRobot class.
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
 * _world: The CoverageWorld representing the environment
 * _fov: The robot's FOV represented as a vector of relative grid cells
 */
class RandomCoverageRobot : public CoverageRobot {

private:
  std::shared_ptr<CoverageWorld> _world{};
  const std::vector<GridCell> _fov{};

  /**
   * Function for printing current transition to stdout.
   *
   * @param startLoc The robot's location at the start of the transition
   * @param outcome The action outcome
   */
  void _printCurrentTransition(const GridCell &startLoc,
                               const ActionOutcome &outcome);

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
   * @return nextAction The next action to be executed
   */
  Action _planFn(const GridCell &currentLoc,
                 const std::vector<Action> &enabledActions, int ts,
                 int timeBound, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &visited,
                 const std::vector<IMacObservation> &currentObs);

  /**
   * Executes an action by checking against the IMacExecutor.
   * Just applies the action to the grid and checks the outcome.
   *
   * @param currentLoc The robot's current location
   * @param action The action to execute
   *
   * @return outcome The outcome of the action
   */
  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action);

  /**
   *  Dummy observation function which returns an empty vector.
   *
   * @param currentLoc The robot's current location
   *
   * @return obsVector A vector of observations
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
   */
  RandomCoverageRobot(const GridCell &currentLoc, int timeBound, int xDim,
                      int yDim, std::shared_ptr<CoverageWorld> world,
                      const std::vector<GridCell> &fov,
                      std::shared_ptr<IMac> groundTruthIMac = nullptr)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim, groundTruthIMac},
        _world{world}, _fov{fov} {
    this->resetForNextEpisode(currentLoc, timeBound);
  }

  /**
   * Resets all necessary members for the next episode.
   *
   * @param startLoc The robot's initial location for the episode
   * @param timeBound The episode time bound, which could change
   */
  void resetForNextEpisode(const GridCell &startLoc, int timeBound);
};

#endif