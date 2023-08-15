/**
 * Small experiment testing planning performance over multiple episodes.
 *
 * @author Charlie Street
 */
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/pomdp_coverage_robot.h"
#include "coverage_plan/util/seed.h"
#include <Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

/**
 * Create a random IMac instance for this example.
 *
 * @return imac A shared pointer to an IMac instance
 */
std::shared_ptr<IMac> createIMac() {
  Eigen::MatrixXd entryMatrix{4, 4};
  Eigen::MatrixXd exitMatrix{4, 4};
  Eigen::MatrixXd initialBelief{4, 4};

  int numSet{0};
  const int staticObsLimit{3};
  const int staticFreeLimit{9};
  const int semiStaticLimit{13};

  // Random order of cells on grid map
  std::vector<GridCell> cells{};
  for (int x{0}; x < 4; ++x) {
    for (int y{0}; y < 4; ++y) {
      cells.push_back(GridCell{x, y});
    }
  }
  std::mt19937_64 rng{SeedHelpers::genRandomDeviceSeed()};
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
 * Write out the multi-episode results.
 *
 * @param results The results for each multi-episode repeat
 * @param outFile The file to write out to
 */
void writeResults(const std::vector<std::vector<double>> &results,
                  const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    for (const std::vector<double> &currentRes : results) {
      for (const double &propCovered : currentRes) {
        f << propCovered << ',';
      }
    }
    f << '\n';
  }
  f.close();
}

int main() {

  std::shared_ptr<IMac> imac{createIMac()};

  // The robot's field of view
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  GridCell initPos{0, 0};
  int timeBound{25};
  int numRepeats{2};  // 10
  int numEpisodes{2}; // 200

  std::vector<std::vector<double>> results{};

  for (int i{0}; i < numRepeats; ++i) {
    std::vector<double> currentRes{};

    // Start from scratch for each repeat
    std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

    std::unique_ptr<POMDPCoverageRobot> robot{
        std::make_unique<POMDPCoverageRobot>(initPos, timeBound, 4, 4, fov,
                                             exec)};

    for (int episode{0}; episode < numEpisodes; ++episode) {
      std::cout << "Repeat: " << i << "; Episode: " << episode << '\n';

      // Write output logs to dummy file
      currentRes.push_back(
          robot->runCoverageEpisode("/tmp/episodeVisited.csv"));
    }
  }

  return 0;
}
