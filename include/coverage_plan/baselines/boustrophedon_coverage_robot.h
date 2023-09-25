/**
 * Coverage robot which follows a boustrophedon motion.
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
 * Members: As in superclass
 */
class BoustrophedonCoverageRobot : public POMDPCoverageRobot {

private:
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
   */
  BoustrophedonCoverageRobot(const GridCell &currentLoc, int timeBound,
                             int xDim, int yDim,
                             const std::vector<GridCell> &fov,
                             std::shared_ptr<IMacExecutor> exec)
      : POMDPCoverageRobot(currentLoc, timeBound, xDim, yDim, fov, exec) {}
};

#endif