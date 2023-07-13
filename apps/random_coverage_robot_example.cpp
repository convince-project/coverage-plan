/**
 * Example script which runs a random coverage robot.
 *
 * @author: Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/coverage_robot.h"
#include <Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <tuple>
#include <vector>

/**
 * Synthesises a random valid action for the coverage robot.
 * Recall that x goes from left to right, y from top to bottom.
 *
 * @param currentLoc The robot's current location
 * @param ts The current timestep
 * @param timeBound The time bound for planning
 * @param imac The IMac instance
 * @param covered The covered nodes
 * @param obsVector The current set of observations
 *
 * @return action The action to execute next
 */
Action randomAction(const GridCell &currentLoc, int ts, int timeBound,
                    std::shared_ptr<IMac> imac,
                    const std::vector<GridCell> &covered,
                    const std::vector<IMacObservation> &obsVector) {

  std::vector<Action> validActions{Action::wait};

  // Used to check dimensions of map
  Eigen::MatrixXd entryMat{imac->getEntryMatrix()};

  // up: y-1
  if (currentLoc.y - 1 >= 0) {
    validActions.push_back(Action::up);
  }

  // down: y+1
  if (currentLoc.y + 1 < entryMat.rows()) {
    validActions.push_back(Action::down);
  }

  // left: x-1
  if (currentLoc.x - 1 >= 0) {
    validActions.push_back(Action::left);
  }

  // right: x+1
  if (currentLoc.x + 1 < entryMat.cols()) {
    validActions.push_back(Action::right);
  }

  std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<> sampler{0, validActions.size() - 1};

  return validActions.at(sampler(gen));
}

/**
 * Prints the current transition in the example run.
 *
 * @param startLoc where did the robot start the transition?
 * @param outcome The action outcome
 */
void printCurrentTransition(const GridCell &startLoc,
                            const ActionOutcome &outcome) {
  std::cout << "STATE: (" << startLoc.x << ',' << startLoc.y << '); ACTION: ';
  switch (outcome.action) {
  case Action::up:
    std::cout << "up";
    break;
  case Action::down:
    std::cout << "down";
    break;
  case Action::left:
    std::cout << "left";
    break;
  case Action::right:
    std::cout << "right";
    break;
  case Action::wait:
    std::cout << "wait";
    break;
  }

  std::cout << "; SUCCESS: " << outcome.success << "SUCCESSOR: (";
  std::cout << outcome.location.x << ',' << outcome.location.y << ")\n";
}

/**
 * The action execution function.
 * Just applies the action to the grid and checks the outcome.
 *
 * @param executor The IMac executor
 * @param map The map data over time
 * @param currentLoc The robot's current location
 * @param action The robot's action to execute
 *
 * @return outcome The ActionOutcome object describing the outcome
 */
ActionOutcome
execute(std::shared_ptr<IMacExecutor> executor,
        const std::vector<std::vector<std::pair<GridCell, int>>> &map,
        const GridCell &currentLoc, const Action &action) {
  // Apply action
  GridCell nextLoc{};
  switch (action) {
  case Action::up:
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y - 1;
    break;
  case Action::down:
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y + 1;
    break;
  case Action::left:
    nextLoc.x = currentLoc.x - 1;
    nextLoc.y = currentLoc.y;
    break;
  case Action::right:
    nextLoc.x = currentLoc.x + 1;
    nextLoc.y = currentLoc.y;
    break;
  case Action::wait:
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y;
    break;
  }

  // Unroll the next IMac state here so we can check occlusion
  Eigen::MatrixXi nextState{
      executor->updateState(std::vector<IMacObservation>{})};
  // Log the difference in map state
  logMapDiff(map, nextState);

  bool succ{true};
  // Check for action failure (note flipped x and y)
  if (nextState[nextLoc.y, nextLoc.x] == 1 && action != Action::wait) {
    succ = false;
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y;
  }

  ActionOutcome outcome{action, succ, nextLoc};

  printCurrentTransition(currentLoc, outcome);

  return outcome;
}

/**
 * Dummy observation function which returns an empty vector.
 *
 * @param currentLoc The robot's current location
 */
std::vector<IMacObservation> observe(const GridCell &currentLoc) {
  return std::vector<IMacObservation>{};
}

/**
 * Creates the robot, wrapping the execute function to handle the internals.
 *
 * @param executor The IMac executor
 * @param map The vector describing the realisation of the map
 *
 * @return robot A pointer to a CoverageRobot object
 */
std::shared_ptr<CoverageRobot>
createRobot(std::shared_ptr<IMacExecutor> executor,
            const std::vector<std::vector<std::pair<GridCell, int>>> &map) {

  // Pass in by reference but match CoverageRobot type definition
  auto executeLambda{[&](const GridCell &currentLoc, const Action &action) {
    return execute(executor, map, currentLoc, action);
  }};
  return std::make_shared<CoverageRobot>(GridCell{0, 0}, 100, 10, 10,
                                         randomAction, executeLambda, observe);
}

/**
 * Create a random IMac instance for this example.
 *
 * @return imac A shared pointer to an IMac instance
 */
std::shared_ptr<IMac> createIMac() {
  Eigen::MatrixXd entryMatrix{10, 10};
  Eigen::MatrixXd exitMatrix{10, 10};
  Eigen::MatrixXd initialBelief{10, 10};

  int numSet{0};
  const int staticObsLimit{20};
  const int staticFreeLimit{50};
  const int semiStaticLimit{75};

  // Random order of cells on grid map
  std::vector<GridCell> cells{};
  for (int x{0}; x < 10; ++x) {
    for (int y{0}; y < 10; ++y) {
      cells.push_back(GridCell{x, y});
    }
  }
  std::mt19937 rng{std::random_device{}()};
  std::shuffle(std::begin(cells), std::end(cells), rng);

  // Have to use y,x to match coordinate systems up
  for (const GridCell &cell : cells) {
    if (numSet < staticObsLimit) {
      entryMatrix(cell.y, cell.x) = 1.0;
      exitMatrix(cell.y, cell.x) = 0.0;
      initialBelief(cell.y, cell.x) = 1.0;
    } else if (numSet < staticFreeLimit) {
      entryMatrix(cell.y, cell.x) = 0.0;
      exitMatrix(cell.y, cell.x) = 1.0;
      initialBelief(cell.y, cell.x) = 0.0;
    } else if (numSet < semiStaticLimit) {
      entryMatrix(cell.y, cell.x) = 0.05;
      exitMatrix(cell.y, cell.x) = 0.05;
      initialBelief(cell.y, cell.x) = 0.3;
    } else {
      entryMatrix(cell.y, cell.x) = 0.5;
      exitMatrix(cell.y, cell.x) = 0.5;
      initialBelief(cell.y, cell.x) = 0.5;
    }
    ++numSet;
  }

  return std::make_shared<IMac>(entryMatrix, exitMatrix, initialBelief);
}

/**
 * Log how the map has changed in the last time step.
 *
 * @param map The current vector of how the map has changed so far
 * @param state The current IMac state
 */
void logMapDiff(const std::vector<std::vector<std::pair<GridCell, int>>> &map,
                const Eigen::MatrixXi &state) {
  std::vector<std::pair<GridCell, int>> mapAtTs{};

  for (int y{0}; y < state.rows(); ++y) {
    for (int x{0}; x < state.cols(); ++x) {
      mapAtTs.push_back(std::make_pair(GridCell{x, y}, state(y, x)));
    }
  }
}

/**
 * Output the map information into a csv file.
 *
 * Each row will be the differences in the map in that timestep.
 * Row format: ts,(x,y,occ)*
 *
 * @param map A vector of vectors of GridCell,occupied pairs
 * @param outFile The CSV file to write everything
 */
void logMap(std::vector<std::vector<std::pair<GridCell, int>>> map,
            std::filesystem::path outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    int ts{0};
    for (const std::vector<std::pair<GridCell, int>> &mapAtTs : map) {
      f << ts << ',';
      for (const std::pair<GridCell, int> &cellVal : mapAtTs) {
        f << std::get<0>(cellVal).x << ',' << std::get<0>(cellVal).y << ','
          << std::get<1>(cellVal) << ',';
      }
      f << '\n';
      ++ts;
    }
  }
  f.close();
}

int main() {

  // Create the IMac Executor
  std::shared_ptr<IMacExecutor> executor{
      std::make_shared<IMacExecutor>(createIMac())};

  // Used for logging the map's observed dynamics
  std::vector<std::vector<std::pair<GridCell, int>>> map{};

  std::shared_ptr<CoverageRobot> coverageRobot{createRobot(executor, map)};

  // Initial IMac state
  Eigen::MatrixXi initState{executor->restart()};
  logMapDiff(map, initState);

  // Start example
  coverageRobot->runCoverageEpisode(
      "/home/charlie/work/coverage-plan/data/results/"
      "randomCoverageRobotExampleCovered.csv");

  // Log the map over time
  logMap(map, "/home/charlie/work/cover-plan/data/results/"
              "randomCoverageRobotExampleMap.csv");

  return 0;
}
