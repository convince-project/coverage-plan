/**
 * Header file for the POMDPCoverageRobot class.
 *
 * This is an implementation of CoverageRobot which uses the POMDP planner.
 *
 * @author Charlie Street
 */
#ifndef POMDP_COVERAGE_ROBOT_H
#define POMDP_COVERAGE_ROBOT_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/coverage_planner.h"
#include "coverage_plan/planning/coverage_robot.h"
#include <memory>
#include <vector>
/**
 * A class for a coverage robot which uses the POMDP planner.
 *
 * Members:
 * As in superclass, plus:
 */
class POMDPCoverageRobot : public CoverageRobot {

private:
  std::shared_ptr<IMacExecutor> _exec{};
  const std::vector<GridCell> _fov{};
  std::vector<IMacObservation> _latestObs{};
  std::unique_ptr<CoveragePlanner> _planner{};

  /**
   * Synthesises an action using the POMDP planner
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
   * Executes an action using a CoverageWorld object.
   * Just applies the action to the grid and checks the outcome.
   *
   * @param currentLoc The robot's current location
   * @param action The action to execute
   *
   * @return outcome The outcome of the action
   */
  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action);

  /**
   *  Returns the latest observation obtained from the CoverageWorld.
   *
   * @param currentLoc The robot's current location
   *
   * @return obsVector A vector of observations
   */
  std::vector<IMacObservation> _observeFn(const GridCell &currentLoc);

public:
  /**
   * Constructor calls super constructor and initialises the planner.
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
  POMDPCoverageRobot(const GridCell &currentLoc, int timeBound, int xDim,
                     int yDim, const std::vector<GridCell> &fov,
                     std::shared_ptr<IMacExecutor> exec,
                     std::shared_ptr<IMac> groundTruthIMac = nullptr)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim, groundTruthIMac},
        _exec{exec}, _fov{fov}, _latestObs{}, _planner{nullptr} {
    this->resetForNextEpisode(currentLoc, timeBound);
  }

  /**
   * Destructor calls resetForNextEpisode which will hapndle memory management.
   * TODO: Not convinced by this
   */
  ~POMDPCoverageRobot() { this->resetForNextEpisode(GridCell{0, 0}, 0); }

  /**
   * Resets all necessary members for the next episode.
   *
   * @param startLoc The robot's initial location for the episode
   * @param timeBound The episode time bound, which could change
   */
  void resetForNextEpisode(const GridCell &startLoc, int timeBound);
};

#endif