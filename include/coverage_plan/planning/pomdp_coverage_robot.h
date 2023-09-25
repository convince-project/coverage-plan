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
#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/planning/coverage_planner.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/planning/coverage_robot.h"
#include "coverage_plan/planning/coverage_world.h"
#include <despot/core/solver.h>
#include <memory>
#include <string>
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
  CoveragePOMDP *_pomdp{};
  CoverageWorld *_world{};
  despot::Solver *_solver{};
  const std::string _boundType{};
  const double _pruningConstant{};

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

  /**
   * Computes initial observation, which is different to others.
   *
   * @param startLoc The robot's start location
   *
   * @return initObs The initial observation as a vector of IMacObservations
   */
  std::vector<IMacObservation> _initialObservation(const GridCell &startLoc);

protected: // Protected members are needed for subclassing
  CoverageBelief *_belief{};

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

public:
  /**
   * Constructor calls super constructor and initialises new members.
   *
   * @param currentLoc The robot's current location
   * @param timeBound The planning time bound
   * @param xDim The x dimension of the map
   * @param yDim The y dimension of the map
   * @param exec The IMacExecutor representing the environment
   * @param fov The robot's FOV as a vector of relative grid cells
   * @param groundTruthIMac The ground truth IMac instance (if we don't want to
   * use BiMac)
   * @param boundType The type of upper and lower bounds to use
   * @param pruningConstant The DESPOT pruning constant
   */
  POMDPCoverageRobot(const GridCell &currentLoc, int timeBound, int xDim,
                     int yDim, const std::vector<GridCell> &fov,
                     std::shared_ptr<IMacExecutor> exec,
                     std::shared_ptr<IMac> groundTruthIMac = nullptr,
                     std::string boundType = "DEFAULT",
                     const double &pruningConstant = 0.01)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim, groundTruthIMac},
        _exec{exec}, _fov{fov}, _latestObs{}, _planner{nullptr},
        _pomdp{nullptr}, _world{nullptr}, _belief{nullptr}, _solver{nullptr},
        _boundType{boundType}, _pruningConstant{pruningConstant} {}

  /**
   * Ensures everything is cleaned up on object deletion.
   * episodeCleanup will only deallocate if not nullptrs.
   */
  ~POMDPCoverageRobot() { this->episodeCleanup(); }

  /**
   * Runs necessary setup for creating the POMDP planner.
   *
   * @param startLoc The robot's initial location for the episode
   * @param ts The initial timestep
   * @param timeBound The episode time bound, which could change
   * @param imacForEpisode The IMac instance being used for the planning episode
   */
  void episodeSetup(const GridCell &startLoc, const int &ts,
                    const int &timeBound, std::shared_ptr<IMac> imacForEpisode);

  /**
   * Deallocates objects used by the CoveragePlanner object.
   */
  void episodeCleanup();
};

#endif