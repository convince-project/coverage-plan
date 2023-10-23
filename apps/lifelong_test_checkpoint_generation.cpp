/**
 * Script which checkpoints the MLE iMac model at different points during
 * training.
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
#include <sys/stat.h>
#include <vector>

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
    runFiles.push_back(inDir / ("episode_" + std::to_string(r) + ".csv"));
  }
  return std::make_shared<FixedIMacExecutor>(runFiles, dim.first, dim.second);
}

/**
 * Checkpoint the BiMac MLE at a given episode.
 *
 * @param type The learning type
 * @param episode The episode number
 * @param baseDir The base directory for the checkpoints
 * @param imacEstimate The imacEstimate to write out
 */
void checkpointIMac(const ParameterEstimate &type, const int &episode,
                    const std::filesystem::path &baseDir,
                    std::shared_ptr<IMac> imacEstimate) {
  std::filesystem::path imacDir{baseDir};

  if (type == ParameterEstimate::posteriorSample) {
    imacDir /= "posterior_sampling";
  } else if (type == ParameterEstimate::maximumLikelihood) {
    imacDir /= "maximum_likelihood";
  }

  imacDir /= "episode_" + std::to_string(episode);

  if (mkdir(imacDir.c_str(), S_IRWXU) == -1) {
    std::cerr << "Error creating directory " << imacDir << '\n';
    exit(1);
  }

  // write the IMac instance
  imacEstimate->writeIMac(imacDir);
}

void runiMacCheckpointer(const std::filesystem::path &imacDir,
                         const std::filesystem::path &baseDir,
                         const int &timeBound, const int &xDim,
                         const int &yDim) {

  // The robot's field of view
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{0, -1}, GridCell{1, -1},
                            GridCell{-1, 0},  GridCell{1, 0},  GridCell{-1, 1},
                            GridCell{0, 1},   GridCell{1, 1}};

  GridCell initPos{0, 0};
  int numEpisodes{150};

  std::vector<ParameterEstimate> methods{ParameterEstimate::posteriorSample,
                                         ParameterEstimate::maximumLikelihood};
  std::set<int> toCheckpoint{0, 1, 5, 10, 50, 100, 150};

  for (const ParameterEstimate &method : methods) {

    // Start from scratch for each method
    // Episodes will be played in same order
    std::shared_ptr<FixedIMacExecutor> exec{
        getExecutor(imacDir, std::make_pair(xDim, yDim), numEpisodes)};

    std::unique_ptr<POMDPCoverageRobot> robot{
        std::make_unique<POMDPCoverageRobot>(initPos, timeBound, xDim, yDim,
                                             fov, exec, nullptr, method)};

    // Get initial error
    std::shared_ptr<IMac> estimate{robot->getBIMac()->mle()};
    checkpointIMac(method, 0, baseDir, estimate);

    for (int episode{1}; episode <= numEpisodes; ++episode) {
      if (method == ParameterEstimate::posteriorSample) {
        std::cout << "Method: Posterior Sampling; Episode: " << episode << '\n';
      } else if (method == ParameterEstimate::posteriorMean) {
        std::cout << "Method: Posterior Mean; Episode: " << episode << '\n';
      } else if (method == ParameterEstimate::maximumLikelihood) {
        std::cout << "Method: Maximum Likelihood; Episode: " << episode << '\n';
      }

      // Write output logs to dummy file
      robot->runCoverageEpisode("/tmp/episodeVisited.csv");

      if (toCheckpoint.count(episode) == 1) {
        std::cout << "CHECKPOINTING CURRENT BIMAC MLE\n";
        // Get current iMac error using BiMac MLE estimate
        estimate = robot->getBIMac()->mle();
        checkpointIMac(method, episode, baseDir, estimate);
      }
    }
  }
}

int main() {

  std::cout << "Running for 5x5 very heavy env\n";
  std::filesystem::path imacDir{
      "../../data/prelim_exps/lifelong_test/five_very_heavy"};
  std::filesystem::path baseDir{
      "../../data/prelim_exps/checkpoints/five_very_heavy"};
  int timeBound{33};
  int xDim{5};
  int yDim{5};
  runiMacCheckpointer(imacDir, baseDir, timeBound, xDim, yDim);

  std::cout << "Running for 7x7 very heavy env\n";
  imacDir = "../../data/prelim_exps/lifelong_test/seven_very_heavy";
  baseDir = "../../data/prelim_exps/checkpoints/seven_very_heavy";
  timeBound = 64;
  xDim = 7;
  yDim = 7;
  runiMacCheckpointer(imacDir, baseDir, timeBound, xDim, yDim);
  return 0;
}
