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
#include "coverage_plan/mod/imac_executor.h"
#include <filesystem>
#include <memory>
#include <vector>

/**
 * Enum for robot actions on a grid.
 */
enum class Action { up, down, left, right, wait };

/**
 * Struct for storing action outcomes.
 *
 * Action outcome contains action success/failure flag, the successor
 * location, and a reminder of the action executed.
 *
 * Members:
 * action: The action executed
 * success: A flag set to true if the action was successful
 * location: The current GridCell of the robot
 */
struct ActionOutcome {
  Action action{};
  bool success{};
  GridCell location{};
};

/**
 * A class for the coverage planning execution framework.
 *
 * Handles the plan/execute/observe cycle.
 *
 * Members:
 * _currentLoc: The robot's current GridCell
 * _covered: A vector of locations covered by the robot
 * _timeBound: The maximum number of timesteps given to the robot
 * _bimac: The BIMac model the robot is learning
 * _planFn: A function which takes the robot's location, the timestep, the
 * timeBound, IMac instance, covered vector, and current observations, and
 * returns an action
 * _executeFn: A function which takes the robot's current location and action
 * and returns an ActionOutcome
 * _observeFn: A function which takes the robot's
 * current location and returns a vector of IMacObservations
 */
class CoverageRobot {
private:
  GridCell _currentLoc{};
  std::vector<GridCell> _covered{};
  int _timeBound{};
  std::unique_ptr<BIMac> _bimac{};
  const std::function<Action(GridCell, int, int, std::shared_ptr<IMac>,
                             std::vector<GridCell>,
                             std::vector<IMacObservation>)>
      _planFn{};
  const std::function<ActionOutcome(GridCell, Action)> _executeFn{};
  const std::function<std::vector<IMacObservation>(GridCell)> _observeFn{};

public:
  /**
   * Initialises all member variables.
   *
   * @param currentLoc The robot's current grid cell
   * @param timeBound: The maximum number of timesteps given to the robot
   * @param xDim The length of the x dimension of the environment
   * @param yDim The length of the y dimension of the environment
   * @param planFn: A function which takes the robot's location, the timestep,
   * the timeBound, IMac instance, covered vector, and current observations, and
   * returns an action
   * @param executeFn: A function which takes the robot's current location and
   * action and returns an ActionOutcome
   * @param observeFn: A function which takes the robot's current location and
   * returns a vector of IMacObservations
   */
  CoverageRobot(
      const GridCell &currentLoc, int timeBound, int xDim, int yDim,
      const std::function<Action(GridCell, int, int, std::shared_ptr<IMac>,
                                 std::vector<GridCell>,
                                 std::vector<IMacObservation>)> &planFn,
      const std::function<ActionOutcome(GridCell, Action)> &executeFn,
      const std::function<std::vector<IMacObservation>(GridCell)> &observeFn)
      : _currentLoc{currentLoc}, _covered{}, _timeBound{timeBound},
        _bimac{std::make_unique<BIMac>(xDim, yDim)}, _planFn{planFn},
        _executeFn{executeFn}, _observeFn{observeFn} {}

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
                        std::vector<IMacObservation> obsVector);

  /**
   * Wrapper around _executeFn which fills in the gaps from class members.
   *
   * @param action The action the robot should execute
   *
   * @return outcome The outcome of the action
   */
  ActionOutcome executeAction(Action action);

  /**
   * Wrapper around _observeFn which fills in the gaps from class members.
   *
   * @return obsVector A vector of observations
   */
  std::vector<IMacObservation> makeObservations();

  /**
   * Resets all necessary members for the next episode.
   */
  void resetForNextEpisode();

  /**
   * Logs a set of covered nodes to file
   *
   * @param outFile The csv file to write the covered locations to
   */
  void logCoveredLocations(std::filesystem::path outFile);

  /**
   * Run the plan-execute-observe cycle for a single episode, up to _timeBound.
   *
   * @param outFile The csv file to output covered locations to
   */
  void runCoverageEpisode(std::filesystem::path outFile);
};

#endif