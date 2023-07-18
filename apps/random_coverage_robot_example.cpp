/**
 * Example script which runs a random coverage robot.
 *
 * @author: Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/random_coverage_robot.h"
#include <Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>

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
  const int staticFreeLimit{75};
  const int semiStaticLimit{90};

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

int main() {

  // Create the IMac Executor
  // Allocating on heap but then just using reference to object
  std::shared_ptr<IMacExecutor> executor{
      std::make_shared<IMacExecutor>(createIMac())};

  std::shared_ptr<RandomCoverageRobot> coverageRobot{
      std::make_shared<RandomCoverageRobot>(GridCell{5, 5}, 100, 10, 10,
                                            executor)};

  // Initial IMac state
  Eigen::MatrixXi initState{executor->restart()};

  // Start example
  coverageRobot->runCoverageEpisode(
      "/home/charlie/work/coverage-plan/data/results/"
      "randomCoverageRobotExampleCovered.csv");

  // Log the map over time
  executor->logMapDynamics("/home/charlie/work/coverage-plan/data/results/"
                           "randomCoverageRobotExampleMap.csv");

  return 0;
}
