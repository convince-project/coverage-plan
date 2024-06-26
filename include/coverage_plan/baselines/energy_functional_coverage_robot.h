/**
 *
 * @file energy_functional_coverage_robot.h
 *
 * @brief Implements energy functional coverage approach
 *
 * Coverage robot which uses the energy functional approach in:
 *
 * New brooms sweep clean - An autonomous robotic cleaning assistant for
 * professional office cleaning
 *
 * Indoor Coverage Path Planning: Survey, Implementation, Analysis
 *
 * A reference implementation can be found here:
 * https://github.com/ipa320/ipa_coverage_planning/blob/noetic_dev/ipa_room_exploration/common/src/energy_functional_explorator.cpp
 *
 * Some modifications have been necessary from the above implementation to
 * handle dynamic environments, partial observability, and 4-connected grids
 * We also ignore the robot's neighbourhood when generating the uncovered cells.
 * Mods:
 * 1. The N term and wall points term are adjusted to 4-connected grids. I.e. N
 * is 1/2(4-num visited), and the wall point term is 0.36 - 0.09 * wall points.
 * 2. To handle partial observability, we ignore the occupied status when we
 * evaluate E, as we can only see the immediate neighbours
 * 3. When we compute the uncovered nodes, we check no neighbours are included.
 * This is because a node may be unvisited, but currently occluded. However,
 * this node may change in the future so it may still be visited eventually.
 * 4. In dynamic environments, we forbid actions which lead to cells *currently*
 * occupied.
 * 5. A robot may only wait if it has finished covering all nodes.
 * 6. The rotational term in the energy function is always 0, as the robot is
 * holonomic with a 360 degree FoV.
 *
 * @author Charlie Street
 */
#ifndef ENERGY_FUNCTIONAL_COVERAGE_ROBOT_H
#define ENERGY_FUNCTIONAL_COVERAGE_ROBOT_H

#include "coverage_plan/planning/pomdp_coverage_robot.h"

/**
 * Subclass of POMDPCoverageRobot which instead uses the energy functional
 * method in 'New brooms sweep clean - An autonomous robotic cleaning assistant
 * for professional office cleaning'.
 *
 * At each time step, the robot chooses a target which minimises an energy
 * function. If the target is not a neighbour, we choose the neighbour which
 * gets us closest.
 *
 * Members: As in superclass, plus:
 * * _useWallPointTerm: A boolean flag. If true, we use the wall point term that
 * appears in the reference implementation, but not the relevant papers.
 */
class EnergyFunctionalCoverageRobot : public POMDPCoverageRobot {
private:
  const bool _useWallPointTerm{};

  /**
   * Get the neighbours of cell.
   * The neighbours are the 4 connected cells around cell.
   * Neighbours may be less than 4 if at the boundary of the map.
   *
   * @param cell The cell to get the neighbours for
   *
   * @param neighbours The vector of neighbours
   */
  std::vector<GridCell> _getNeighbours(const GridCell &cell);

  /**
   * Get the uncovered cells, that aren't in the robot's neighbourhood.
   *
   * We've already established that the robot's immediate neighbourhood can't
   * be moved to if we are calling this function. So this should get unvisited
   * nodes outside of that.
   *
   * NOTE: This differs slightly from the original paper.
   *
   * @param visited The currently visited nodes
   *
   * @returns A vector of uncovered cells.
   */
  std::vector<GridCell> _getUncovered(const GridCell &currentCell,
                                      const std::vector<GridCell> &visited);

  /**
   * Computes the manhattan distance between two grid cells.
   *
   * @param a GridCell one
   * @param b GridCell two
   *
   * @returns The manhattan distance between a and b
   */
  int _manhattanDistance(const GridCell &a, const GridCell &b);

  /**
   * Computes the energy function for the current grid cell and possible
   * successor.
   *
   * @param currentCell The current GridCell
   * @param nextCell The possible next GridCell
   * @param visited The vector of visited locations
   *
   * @returns The energy value for this current cell/next cell pair
   */
  double _E(const GridCell &currentCell, const GridCell &nextCell,
            const std::vector<GridCell> &visited);

  /**
   * Chooses the action which minimises the energy functional.
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
   * @param useWallPointTerm If true, we use the wall point term that
   * appears in the reference implementation, but not the relevant papers.
   */
  EnergyFunctionalCoverageRobot(const GridCell &currentLoc, int timeBound,
                                int xDim, int yDim,
                                const std::vector<GridCell> &fov,
                                std::shared_ptr<IMacExecutor> exec,
                                std::shared_ptr<IMac> groundTruthIMac = nullptr,
                                const ParameterEstimate &estimationType =
                                    ParameterEstimate::posteriorSample,
                                bool useWallPointTerm = true)
      : POMDPCoverageRobot(currentLoc, timeBound, xDim, yDim, fov, exec,
                           groundTruthIMac, estimationType),
        _useWallPointTerm{useWallPointTerm} {}
};

#endif
