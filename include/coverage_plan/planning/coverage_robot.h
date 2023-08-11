/**
 * Header file for the CoverageRobot class.
 *
 * This class captures the online planning framework.
 *
 * It will likely be overwritten once we move over to a ROS implementation,
 * where planning will occur as the robot is moving.
 *
 * This still contains most of the framework though.
 *
 * @author Charlie Street
 */
#ifndef COVERAGE_ROBOT_H
#define COVERAGE_ROBOT_H

#include "coverage_plan/mod/bimac.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <filesystem>
#include <map>
#include <memory>
#include <vector>

/**
 * A class for the coverage planning execution framework.
 *
 * Handles the plan/execute/observe cycle.
 *
 * Members:
 * _currentLoc: The robot's current GridCell
 * _visited: A vector of locations visited by the robot (i.e. the history)
 * _timeBound: The maximum number of timesteps given to the robot
 * _xDim: The x dimension of the map
 * _yDim: The y dimension of the map
 * _bimac: The BIMac model the robot is learning
 */
class CoverageRobot {
private:
  GridCell _currentLoc{};
  std::vector<GridCell> _visited{};
  int _timeBound{};
  const int _xDim{};
  const int _yDim{};
  std::shared_ptr<BIMac> _bimac{};
  std::shared_ptr<IMac> _groundTruthIMac{};

  /**
   * Function gets the IMac instance to be used for an episode.
   * In this class, it will be set to generate a posterior sample.
   *
   * @return imac The IMac instance for the coverage episode.
   */
  virtual std::shared_ptr<IMac> _getIMacInstanceForEpisode();

  /**
   * Helper function to add an initial state observation to a BIMacObservation.
   *
   * @param biMacObsMap a map from grid cells to corresponding BIMacObservations
   * @param obs an IMacObservation for time t=0
   */
  void _addInitObservation(std::map<GridCell, BIMacObservation> &biMacObsMap,
                           const IMacObservation &obs);

  /**
   * Helper function to add IMac transition observation to a BIMacObservation.
   *
   * @param biMacObsMap a map from grid cells to corresponding BIMacObservations
   * @param cell The grid cell the observation is for
   * @param prevState The previous state of the grid cell
   * @param nextState The new state of the grid cell
   */
  void
  _addTransitionObservation(std::map<GridCell, BIMacObservation> &biMacObsMap,
                            const GridCell &cell, const int &prevState,
                            const int &nextState);

  /**
   * Converts IMacObservation vectors into a BIMacObservation vector.
   *
   * @param observations a vector of IMacObservation vectors
   *
   * @return biMacObsVector A vector of BIMacObservations
   */
  std::vector<BIMacObservation> _generateBIMacObservations(
      const std::vector<std::vector<IMacObservation>> &observations);

  /**
   * Returns a vector of actions that can be executed from the current location.
   *
   * @return enabled The enabled actions from the current location
   */
  std::vector<Action> _getEnabledActions();

protected:
  // Pure virtual functions making CoverageRobot an abstract base class

  /**
   * Internal function which generates the next action to be executed.
   *
   * Wrapped around by planNextAction, which passes in the parameters.
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
  virtual Action _planFn(const GridCell &currentLoc,
                         const std::vector<Action> &enabledActions, int ts,
                         int timeBound, std::shared_ptr<IMac> imac,
                         const std::vector<GridCell> &visited,
                         const std::vector<IMacObservation> &currentObs) = 0;

  /**
   * Internal function for executing actions.
   *
   * Wrapped around by executeAction, which passes in parameters.
   *
   * @param currentLoc The robot's current location
   * @param action The action to execute
   *
   * @return outcome The outcome of the action
   */
  virtual ActionOutcome _executeFn(const GridCell &currentLoc,
                                   const Action &action) = 0;

  /**
   * Internal function for making observations.
   *
   * Wrapped around by makeObservations, which passes in parameters.
   *
   * @param currentLoc The robot's current location
   *
   * @return obsVector A vector of observations
   */
  virtual std::vector<IMacObservation>
  _observeFn(const GridCell &currentLoc) = 0;

public:
  /**
   * Initialises all member variables.
   *
   * @param currentLoc The robot's current grid cell
   * @param timeBound: The maximum number of timesteps given to the robot
   * @param xDim The length of the x dimension of the environment
   * @param yDim The length of the y dimension of the environment
   * @param groundTruthIMac The ground truth IMac (if we want to test planning
   * without BiMac)
   */
  CoverageRobot(const GridCell &currentLoc, int timeBound, int xDim, int yDim,
                std::shared_ptr<IMac> groundTruthIMac = nullptr)
      : _currentLoc{currentLoc}, _visited{}, _timeBound{timeBound}, _xDim{xDim},
        _yDim{yDim}, _bimac{std::make_shared<BIMac>(xDim, yDim)},
        _groundTruthIMac{groundTruthIMac} {}

  /**
   * Wrapper around _planFn which fills in the gaps from class members.
   *
   * @param time The current timestep
   * @param imac The current imac estimate generated from _bimac
   * @param obsVector A vector of the current observations made by the robot
   *
   * @return action The next action for the robot to execute
   */
  Action planNextAction(int time, std::shared_ptr<IMac> imac,
                        const std::vector<IMacObservation> &obsVector);

  /**
   * Wrapper around _executeFn which fills in the gaps from class members.
   *
   * @param action The action the robot should execute
   *
   * @return outcome The outcome of the action
   */
  ActionOutcome executeAction(const Action &action);

  /**
   * Wrapper around _observeFn which fills in the gaps from class members.
   *
   * @return obsVector A vector of observations
   */
  std::vector<IMacObservation> makeObservations();

  /**
   * Resets all necessary members for the next episode.
   *
   * @param startLoc The robot's initial location for the episode
   * @param timeBound The episode time bound, which could change
   */
  void resetForNextEpisode(const GridCell &startLoc, int timeBound);

  /**
   * Logs a set of visited nodes to file.
   *
   * @param outFile The csv file to write the visited locations to
   */
  void logVisitedLocations(const std::filesystem::path &outFile);

  /**
   * Run the plan-execute-observe cycle for a single episode, up to _timeBound.
   *
   * @param outFile The csv file to output visited locations to
   */
  void runCoverageEpisode(const std::filesystem::path &outFile);

  /**
   * Getter for the BIMac model.
   *
   * Getter useful in case user wishes to write to file etc.
   *
   * @return bimac A shared ptr to a BIMac instance
   */
  std::shared_ptr<BIMac> getBIMac() { return this->_bimac; }
};

#endif