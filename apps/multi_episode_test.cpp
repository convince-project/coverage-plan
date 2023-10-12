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
 * Creates the FixedIMacExecutor.
 *
 * @param inDir The IMac directory
 * @param dim The x,y dimensions of the map
 * @param numRuns The number of runs to read in
 *
 * @return exec The FixedIMacExecutor
 */
std::shared_ptr<FixedIMacExecutor>
getExecutor(const std::filesystem::path &inDir, const std::pair<int, int> &dim,
            const int &numRuns) {
  std::vector<std::filesystem::path> runFiles{};
  for (int r{1}; r <= numRuns; ++r) {
    runFiles.push_back(inDir / ("run_" + std::to_string(r) + ".csv"));
  }
  return std::make_shared<FixedIMacExecutor>(runFiles, dim.first, dim.second);
}

/**
 * Write out the multi-episode results.
 *
 * @param results The results for a given method
 * @param outFile The file to write out to
 */
void writeResults(const std::vector<double> &results,
                  const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    for (const double &propCovered : results) {
      f << propCovered << ',';
    }
    f << '\n';
  }
  f.close();
}

int main() {
  std::filesystem::path imacDir{"../../data/prelim_exps/lifelong_test"};

  // The robot's field of view
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{0, -1}, GridCell{1, -1},
                            GridCell{-1, 0},  GridCell{1, 0},  GridCell{-1, 1},
                            GridCell{0, 1},   GridCell{1, 1}};

  GridCell initPos{0, 0};
  int timeBound{33};
  int numEpisodes{300};

  std::vector<ParameterEstimate> methods{ParameterEstimate::posteriorSample,
                                         ParameterEstimate::maximumLikelihood};

  for (const ParameterEstimate &method : methods) {
    std::vector<double> results{};

    // Start from scratch for each method
    // Episodes will be played in same order
    std::shared_ptr<FixedIMacExecutor> exec{
        getExecutor(imacDir, std::make_pair(5, 5), numEpisodes)};

    std::unique_ptr<POMDPCoverageRobot> robot{
        std::make_unique<POMDPCoverageRobot>(initPos, timeBound, 5, 5, fov,
                                             exec, nullptr, method)};

    for (int episode{1}; episode <= numEpisodes; ++episode) {
      if (method == ParameterEstimate::posteriorSample) {
        std::cout << "Method: Posterior Sampling; Episode: " << episode << '\n';
      } else if (method == ParameterEstimate::posteriorMean) {
        std::cout << "Method: Posterior Mean; Episode: " << episode << '\n';
      } else if (method == ParameterEstimate::maximumLikelihood) {
        std::cout << "Method: Maximum Likelihood; Episode: " << episode << '\n';
      }

      // Write output logs to dummy file
      results.push_back(
          robot->runCoverageEpisode("/tmp/episodeVisited.csv").propCovered);
    }

    if (method == ParameterEstimate::posteriorSample) {
      writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                            "posterior_sample_results.csv");
    } else if (method == ParameterEstimate::posteriorMean) {
      writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                            "posterior_mean_results.csv");
    } else if (method == ParameterEstimate::maximumLikelihood) {
      writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                            "maximum_likelihood_results.csv");
    }
  }

  return 0;
}
