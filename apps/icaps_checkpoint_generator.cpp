/**
 * Checkpoint generator for ICAPS framework experiment.
 *  Only considers posterior mean
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
 * Checkpoint the BiMac posterior mean at a given episode.
 * @param episode The episode number
 * @param baseDir The base directory for the checkpoints
 * @param imacEstimate The imacEstimate to write out
 */
void checkpointIMac(const int &episode, const std::filesystem::path &baseDir,
                    std::shared_ptr<IMac> imacEstimate) {
  std::filesystem::path imacDir{baseDir};
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
  std::set<int> toCheckpoint{0, 1, 5, 10, 50, 100, 150};

  // Start from scratch for each method
  // Episodes will be played in same order
  std::shared_ptr<FixedIMacExecutor> exec{
      getExecutor(imacDir, std::make_pair(xDim, yDim), numEpisodes)};

  std::unique_ptr<POMDPCoverageRobot> robot{
      std::make_unique<POMDPCoverageRobot>(initPos, timeBound, xDim, yDim, fov,
                                           exec, nullptr,
                                           ParameterEstimate::posteriorMean)};

  // Get initial error
  std::shared_ptr<IMac> estimate{robot->getBIMac()->posteriorMean()};
  checkpointIMac(0, baseDir, estimate);

  for (int episode{1}; episode <= numEpisodes; ++episode) {
    std::cout << "Method: Posterior Mean; Episode: " << episode << '\n';
    // Write output logs to dummy file
    robot->runCoverageEpisode("/tmp/episodeVisited.csv");

    if (toCheckpoint.count(episode) == 1) {
      std::cout << "CHECKPOINTING CURRENT BIMAC Posterior Mean\n";
      // Get current iMac error using BiMac MLE estimate
      estimate = robot->getBIMac()->posteriorMean();
      checkpointIMac(episode, baseDir, estimate);
    }
  }
}

/**
 * Generate a number of runs through an IMac model and write to file.
 *
 * @param imac The imac model
 * @param dir The directory to write to
 * @param timeBound The time bound for planning
 * @param numRuns The number of runs to sample
 */
void sampleRuns(std::shared_ptr<IMac> imac, const std::filesystem::path &dir,
                const int &timeBound, const int &numRuns) {

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  for (int run{1}; run <= numRuns; ++run) {
    std::cout << "Generating run " << run << "/" << numRuns << "\n";
    exec->restart(std::vector<IMacObservation>{});
    for (int t{1}; t <= timeBound; ++t) {
      exec->updateState(std::vector<IMacObservation>{});
    }
    exec->logMapDynamics(dir / ("episode_" + std::to_string(run) + ".csv"));
  }
}

void runAllCheckpointing() {
  std::cout << "Running for 6x6 very heavy env\n";
  std::filesystem::path imacDir{
      "../../data/icaps_exps/six_very_heavy/lifelong_samples"};
  std::filesystem::path checkpointDir{
      "../../data/icaps_exps/checkpoints/six_very_heavy"};
  int timeBound{47};
  int xDim{6};
  int yDim{6};
  runiMacCheckpointer(imacDir, checkpointDir, timeBound, xDim, yDim);

  std::cout << "Running for 7x7 very heavy env\n";
  imacDir = "../../data/icaps_exps/seven_very_heavy/lifelong_samples";
  checkpointDir = "../../data/icaps_exps/checkpoints/seven_very_heavy";
  timeBound = 64;
  xDim = 7;
  yDim = 7;
  runiMacCheckpointer(imacDir, checkpointDir, timeBound, xDim, yDim);

  std::cout << "Running for 8x8 very heavy env\n";
  imacDir = "../../data/icaps_exps/eight_very_heavy/lifelong_samples";
  checkpointDir = "../../data/icaps_exps/checkpoints/eight_very_heavy";
  timeBound = 84;
  xDim = 8;
  yDim = 8;
  runiMacCheckpointer(imacDir, checkpointDir, timeBound, xDim, yDim);

  std::cout << "Running for 9x9 very heavy env\n";
  imacDir = "../../data/icaps_exps/nine_very_heavy/lifelong_samples";
  checkpointDir = "../../data/icaps_exps/checkpoints/nine_very_heavy";
  timeBound = 106;
  xDim = 9;
  yDim = 9;
  runiMacCheckpointer(imacDir, checkpointDir, timeBound, xDim, yDim);
}

void runSampleGen() {
  std::filesystem::path sixDir{
      "../../data/icaps_exps/six_very_heavy/lifelong_samples"};
  sampleRuns(std::make_shared<IMac>(sixDir), sixDir, 47, 150);

  std::filesystem::path sevenDir{
      "../../data/icaps_exps/seven_very_heavy/lifelong_samples"};
  sampleRuns(std::make_shared<IMac>(sevenDir), sevenDir, 64, 150);

  std::filesystem::path eightDir{
      "../../data/icaps_exps/eight_very_heavy/lifelong_samples"};
  sampleRuns(std::make_shared<IMac>(eightDir), eightDir, 84, 150);

  std::filesystem::path nineDir{
      "../../data/icaps_exps/nine_very_heavy/lifelong_samples"};
  sampleRuns(std::make_shared<IMac>(nineDir), nineDir, 106, 150);
}

int main() {
  // runSampleGen();
  runAllCheckpointing();
  return 0;
}
