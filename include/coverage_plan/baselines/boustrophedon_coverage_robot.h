/**
 * @file boustrophedon_coverage_robot.h
 *
 * @brief Coverage robot which follows a boustrophedon motion.
 *
 * @author Charlie Street
 */

#ifndef BOUSTROPHEDON_COVERAGE_ROBOT_H
#define BOUSTROPHEDON_COVERAGE_ROBOT_H

#include "coverage_plan/planning/pomdp_coverage_robot.h"

/**
 * Subclass of POMDPCoverageRobot which instead follows a Boustrophedon motion.
 * @see BA*: An online complete coverage algorithm for cleaning robots
 *
 * The boustrophedon motion chooses directions in order up,down,right,left.
 * If no neighbours are free or uncovered, the robot waits.
 *
 * Members: As in superclass, plus:
 * * _waitForObstacles A flag to state if the robot should wait for
 * obstacles to clear (this means the robot follows a fixed path).
 * This is basically offline Boustrophedon
 */
class BoustrophedonCoverageRobot : public POMDPCoverageRobot {

private:
  bool _waitForObstacles{};

  /**
   * Follows a boustrophedon motion online.
   *
   * NOTE: Function assumes that currentObs covers the 4 immediate neighbours
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
   * @param estimationType The type of parameter estimation to use for IMac
   * instance for episode
   * @param waitForObstacles A flag to state if the robot should wait for
   * obstacles to clear (this means the robot follows a fixed path)
   */
  BoustrophedonCoverageRobot(const GridCell &currentLoc, int timeBound,
                             int xDim, int yDim,
                             const std::vector<GridCell> &fov,
                             std::shared_ptr<IMacExecutor> exec,
                             std::shared_ptr<IMac> groundTruthIMac = nullptr,
                             const ParameterEstimate &estimationType =
                                 ParameterEstimate::posteriorSample,
                             bool waitForObstacles = false)
      : POMDPCoverageRobot(currentLoc, timeBound, xDim, yDim, fov, exec,
                           groundTruthIMac, estimationType),
        _waitForObstacles{waitForObstacles} {}
};

#endif