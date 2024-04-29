/**
 * @file test_checkpoints_experiment.cpp
 *
 * @brief Small experiment to test he checkpointed iMac instances.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/fixed_imac_executor.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/coverage_robot.h"
#include "coverage_plan/planning/coverage_world.h"
#include "coverage_plan/planning/pomdp_coverage_robot.h"
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

/**
 * Creates the FixedIMacExecutor.
 *
 * @param imacDir The IMac directory
 * @param dim The x,y dimensions of the map
 * @param numRuns The number of runs to read in
 *
 * @returns The FixedIMacExecutor
 */
std::shared_ptr<FixedIMacExecutor>
getExecutor(const std::filesystem::path &imacDir,
            const std::pair<int, int> &dim, const int &numRuns) {
  std::vector<std::filesystem::path> runFiles{};
  for (int r{1}; r <= numRuns; ++r) {
    runFiles.push_back(imacDir / ("run_" + std::to_string(r) + ".csv"));
  }
  return std::make_shared<FixedIMacExecutor>(runFiles, dim.first, dim.second);
}

/**
 * Write out the results for a given imac model.
 *
 * @param results The results for each environment for a given method
 * @param imacNames The list of imac model names. Order matches results
 * @param outFile The file to write out to
 */
void writeResults(const std::vector<std::vector<double>> &results,
                  const std::vector<std::string> &imacNames,
                  const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    for (int i{0}; i < results.size(); ++i) {
      f << imacNames.at(i) << ',';
      for (const double &propCovered : results.at(i)) {
        f << propCovered << ',';
      }
      f << '\n';
    }
  }
  f.close();
}

/**
 * Runs the experiments for different imac models
 *
 * @param imacs A list of imac models to test
 * @param imacNames The names of the imac models
 * @param fov The robot's field of view
 * @param imacDir The directory to find all of the environment runs/IMac
 * models
 * @param timeBound The planning time bound
 * @param dim The map dimensions
 * @param outFile Where to store the results
 * @param numRuns How many repeats are we running
 */
void runExperiments(const std::vector<std::shared_ptr<IMac>> &imacs,
                    const std::vector<std::string> &imacNames,
                    const std::vector<GridCell> &fov,
                    const std::filesystem::path &imacDir, const int &timeBound,
                    const int &dim, const std::filesystem::path &outFile,
                    const int &numRuns = 10) {

  // Prepare output
  std::vector<std::vector<double>> results{};

  // Iterate through models
  for (int i{0}; i < imacs.size(); ++i) {
    std::cout << "MODEL:" << imacNames.at(i) << "\n";

    // Get run files, fixedIMacExecutor, and ground truth IMac model
    std::shared_ptr<FixedIMacExecutor> exec{
        getExecutor(imacDir, std::make_pair(dim, dim), numRuns)};

    // Get the robot object
    std::shared_ptr<CoverageRobot> robot{std::make_shared<POMDPCoverageRobot>(
        GridCell{0, 0}, timeBound, dim, dim, fov, exec, imacs.at(i))};

    std::vector<double> resultsForModel{};
    for (int r{0}; r < numRuns; ++r) {
      std::cout << "RUN: " << r + 1 << "/" << numRuns << "\n";
      resultsForModel.push_back(
          robot->runCoverageEpisode("/tmp/dummy.csv").propCovered);
    }
    results.push_back(resultsForModel);

    // Output file
    std::cout << "WRITING RESULTS\n";
    writeResults(results, imacNames, outFile);
  }
}

std::pair<std::vector<std::shared_ptr<IMac>>, std::vector<std::string>>
getIMacModels(const std::filesystem::path &checkpointDir,
              const std::filesystem::path &groundTruthDir) {
  // Read in the iMac models
  std::vector<std::shared_ptr<IMac>> imacs{};
  std::vector<std::string> imacNames{};

  std::vector<std::string> learningTypes{"posterior_sampling",
                                         "maximum_likelihood"};
  std::vector<int> checkpoints{0, 1, 5, 10, 50, 100, 150};

  for (const std::string &type : learningTypes) {
    for (const int &checkpoint : checkpoints) {
      std::filesystem::path imacDir{checkpointDir};
      imacDir /= type;
      imacDir /= ("episode_" + std::to_string(checkpoint));

      imacs.push_back(std::make_shared<IMac>(imacDir));
      imacNames.push_back(type + "_episode_" + std::to_string(checkpoint));
    }
  }
  // Add ground truth
  imacs.push_back(std::make_shared<IMac>(groundTruthDir));
  imacNames.push_back("ground_truth");

  return std::make_pair(imacs, imacNames);
}

int main() {

  // Robot FOV
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{0, -1}, GridCell{1, -1},
                            GridCell{-1, 0},  GridCell{1, 0},  GridCell{-1, 1},
                            GridCell{0, 1},   GridCell{1, 1}};

  std::cout << "Running for 5x5 very heavy env\n";
  std::filesystem::path imacDir{"../../data/prelim_exps/five_very_heavy"};
  int numRuns{20};
  int timeBound{33};
  int dim{5};
  auto models{
      getIMacModels("../../data/prelim_exps/checkpoints/five_very_heavy",
                    "../../data/prelim_exps/five_very_heavy")};
  std::filesystem::path outFile{"../../data/results/prelim_exps/"
                                "checkpoint_test/five_very_heavy_results.csv"};
  // Run the experiments
  runExperiments(std::get<0>(models), std::get<1>(models), fov, imacDir,
                 timeBound, dim, outFile, numRuns);

  std::cout << "Running for 7x7 very heavy env\n";
  imacDir = "../../data/prelim_exps/seven_very_heavy";
  timeBound = 64;
  dim = 7;
  models = getIMacModels("../../data/prelim_exps/checkpoints/seven_very_heavy",
                         "../../data/prelim_exps/seven_very_heavy");
  outFile = "../../data/results/prelim_exps/"
            "checkpoint_test/seven_very_heavy_results.csv";
  runExperiments(std::get<0>(models), std::get<1>(models), fov, imacDir,
                 timeBound, dim, outFile, numRuns);
}