/**
 * Small experiment testing planning performance over multiple episodes.
 *
 * @author Charlie Street
 */
#include "coverage_plan/mod/fixed_imac_executor.h"
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
 * Write 300 IMacExecutor episodes for episode
 */
void sampleIMacRuns() {
  std::filesystem::path imacDir{"../../data/prelim_exps/lifelong_test"};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacDir)};
  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  int numEpisodes{300};
  int timeBound{33};

  for (int run{1}; run <= numEpisodes; ++run) {
    std::cout << "Generating run " << run << "/" << numEpisodes << "\n";
    exec->restart(std::vector<IMacObservation>{});
    for (int t{1}; t <= timeBound; ++t) {
      exec->updateState(std::vector<IMacObservation>{});
    }
    exec->logMapDynamics(imacDir / ("episode_" + std::to_string(run) + ".csv"));
  }
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
      f << '\n';
    }
  }
  f.close();
}

int main() {
  sampleIMacRuns();
  exit(1);
  std::shared_ptr<IMac> imac{};

  // The robot's field of view
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  GridCell initPos{0, 0};
  int timeBound{25};
  int numRepeats{10};
  int numEpisodes{100};

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
          robot->runCoverageEpisode("/tmp/episodeVisited.csv").propCovered);
    }

    results.push_back(currentRes);
  }

  writeResults(results, "../../data/results/multiEpisodeTestResults.csv");

  return 0;
}
