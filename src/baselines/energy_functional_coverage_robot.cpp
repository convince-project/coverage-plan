/**
 * Implementation of EnergyFunctionalCoverageRobot in
 * energy_functional_coverage_robot.h.
 * @see energy_functional_coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/baselines/energy_functional_coverage_robot.h"
#include "coverage_plan/mod/grid_cell.h"
#include <algorithm>
#include <stdlib.h>

/**
 * Get the neighbours of cell.
 */
std::vector<GridCell>
EnergyFunctionalCoverageRobot::_getNeighbours(const GridCell &cell) {
  std::vector<GridCell> neighbours{
      GridCell{0, -1} + cell, GridCell{0, 1} + cell, GridCell{-1, 0} + cell,
      GridCell{1, 0} + cell};

  std::vector<GridCell> inBound{};
  for (const GridCell &neighbour : neighbours) {
    if (!neighbour.outOfBounds(0, this->_xDim, 0, this->_yDim)) {
      inBound.push_back(neighbour);
    }
  }

  return inBound;
}

/**
 * Get the uncovered cells.
 */
std::vector<GridCell> EnergyFunctionalCoverageRobot::_getUncovered(
    const std::vector<GridCell> &visited) {
  std::vector<GridCell> uncovered{};
  for (int x{0}; x < this->_xDim; ++x) {
    for (int y{0}; y < this->_yDim; ++y) {
      if (std::count(visited.begin(), visited.end(), GridCell{x, y}) == 0) {
        uncovered.push_back(GridCell{x, y});
      }
    }
  }
  return uncovered;
}

/**
 * Computes the manhattan distance between two grid cells.
 */
int EnergyFunctionalCoverageRobot::_manhattanDistance(const GridCell &a,
                                                      const GridCell &b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

/**
 * Computes the energy function for the current grid cell and possible
 * successor.
 */
double EnergyFunctionalCoverageRobot::_E(const GridCell &currentCell,
                                         const GridCell &nextCell,
                                         const std::vector<GridCell> &visited) {
  // Part 1: Translational distance
  double translateDist{(double)this->_manhattanDistance(currentCell, nextCell)};

  // Part 2: Rotational distance
  // NOTE: Always 0 for holonomic robots with 360 degree FoV
  double rotateDist{0.0};

  // Part 3: N function
  // 1/2(4-num neighbours of nextCell in visited)
  std::vector<GridCell> nextNeighbours{this->_getNeighbours(nextCell)};
  double nTerm{4.0};
  for (const GridCell &neighbour : nextNeighbours) {
    if (std::count(visited.begin(), visited.end(), neighbour) > 0) {
      nTerm -= 1.0;
    }
  }
  nTerm *= 0.5;

  // Part 4: Wall points (if required)
  // 0.36 - 0.09 * numWallPoints
  double wallPointsTerm{0.0};
  if (this->_useWallPointTerm) {
    wallPointsTerm = 0.36 - (0.09 * (4.0 - nextNeighbours.size()));
  }

  return translateDist + rotateDist + nTerm + wallPointsTerm;
}

/**
 * Chooses the action which minimises the energy functional.
 */
Action EnergyFunctionalCoverageRobot::_planFn(
    const GridCell &currentLoc, const std::vector<Action> &enabledActions,
    int ts, int timeBound, std::shared_ptr<IMac> imac,
    const std::vector<GridCell> &visited,
    const std::vector<IMacObservation> &currentObs) {
  // Get hash map of neighbouring observations
  std::map<GridCell, int> obsMap{};
  for (const IMacObservation &imacObs : currentObs) {
    obsMap[currentLoc + imacObs.cell] = imacObs.occupied;
  }

  // Step 1: Get all neighours not currently occluded or visited
  std::vector<GridCell> candidates{};
  for (const Action &action : enabledActions) {
    if (action == Action::wait) {
      continue;
    }
    GridCell nextLoc{ActionHelpers::applySuccessfulAction(currentLoc, action)};
    if (obsMap.count(nextLoc) == 0) {
      throw "[EnergyFunctionalCoverageRobot] Neighbouring direction not "
            "observed\n";
    }

    if (obsMap[nextLoc] == 0 and
        std::count(visited.begin(), visited.end(), nextLoc) == 0) {
      candidates.push_back(nextLoc);
    }
  }

  // Step 2: If no valid neighbours, get every not visited node
  if (candidates.size() == 0) {
    candidates = this->_getUncovered(visited);
  }
  if (candidates.size() == 0) { // If we've covered everything, just wait
    return Action::wait;
  }

  // Step 3: Find cell with minimum energy
  GridCell bestCell{};
  double minE{this->_xDim + this->_yDim + 5.36}; // greater than max energy
  for (const GridCell &candidate : candidates) {
    double energy{this->_E(currentLoc, candidate, visited)};
    if (energy < minE) {
      minE = energy;
      bestCell = candidate;
    }
  }

  // Step 4: Find the best immediate action to take
  Action bestAction{};
  double minDist{this->_xDim + this->_yDim + 1.0}; // Greater than max distance
  for (const Action &action : enabledActions) {
    GridCell nextLoc{ActionHelpers::applySuccessfulAction(currentLoc, action)};
    if (obsMap[nextLoc] == 0) {
      double dist{(double)this->_manhattanDistance(nextLoc, bestCell)};
      if (dist < minDist) {
        minDist = dist;
        bestAction = action;
      }
    }
  }

  return bestAction;
}