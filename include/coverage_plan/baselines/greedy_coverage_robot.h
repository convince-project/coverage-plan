/**
 * Class for robot which chooses actions maximising immediate reward.
 *
 * @author Charlie Street
 */

#ifndef GREEDY_COVERAGE_ROBOT_H
#define GREEDY_COVERAGE_ROBOT_H

#include "coverage_plan/planning/pomdp_coverage_robot.h"

/**
 * Subclass of POMDPCoverageRobot which instead uses a greedy policy.
 *
 * The greedy policy maximises immediate expected reward given a current belief.
 * The immediate expected reward is the probability of the successor cell being
 * free in the next time step, if unvisited (in which case there is no reward.)
 * As its over a belief, not a state, we can't use the implementation in the
 * bounds classes.
 *
 * Members: As in superclass
 */
class GreedyCoverageRobot : public POMDPCoverageRobot {

private:
  /**
   * Greedily selects action which maximises immediate reward.
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
   * Constructor calls super constructor.
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
  GreedyCoverageRobot(const GridCell &currentLoc, int timeBound, int xDim,
                      int yDim, const std::vector<GridCell> &fov,
                      std::shared_ptr<IMacExecutor> exec,
                      std::shared_ptr<IMac> groundTruthIMac = nullptr)
      : POMDPCoverageRobot(currentLoc, timeBound, xDim, yDim, fov, exec,
                           groundTruthIMac, "DEFAULT") {}
};

#endif