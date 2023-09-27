/**
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
 * _useWallPointTerm: A boolean flag. If true, we use the wall point term that
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
   * Get the uncovered cells.
   *
   * @param visited The currently visited nodes
   *
   * @return uncovered A vector of uncovered cells.
   */
  std::vector<GridCell> _getUncovered(const std::vector<GridCell> &visited);

  /**
   * Computes the manhattan distance between two grid cells.
   *
   * @param a GridCell one
   * @param b GridCell two
   *
   * @return dist The manhattan distance between a and b
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
   * @return energy The energy value for this current cell/next cell pair
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
   * @param useWallPointTerm If true, we use the wall point term that
   * appears in the reference implementation, but not the relevant papers.
   */
  EnergyFunctionalCoverageRobot(const GridCell &currentLoc, int timeBound,
                                int xDim, int yDim,
                                const std::vector<GridCell> &fov,
                                std::shared_ptr<IMacExecutor> exec,
                                bool useWallPointTerm = true)
      : POMDPCoverageRobot(currentLoc, timeBound, xDim, yDim, fov, exec),
        _useWallPointTerm{useWallPointTerm} {}
};

#endif
