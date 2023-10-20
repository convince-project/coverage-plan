/**
 * Small experiment to test he checkpointed iMac instances.
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
 * @param inDir The IMac directory
 * @param env The name of the environment
 * @param dim The x,y dimensions of the map
 * @param numRuns The number of runs to read in
 *
 * @return exec The FixedIMacExecutor
 */
std::shared_ptr<FixedIMacExecutor>
getExecutor(const std::filesystem::path &inDir, const std::string &env,
            const std::pair<int, int> &dim, const int &numRuns) {
  std::vector<std::filesystem::path> runFiles{};
  for (int r{1}; r <= numRuns; ++r) {
    runFiles.push_back(inDir / env / ("run_" + std::to_string(r) + ".csv"));
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
 * @param inDir The directory to find all of the environment runs/IMac
 * models
 * @param numRuns How many repeats are we running
 */
void runExperiments(const std::vector<std::shared_ptr<IMac>> &imacs,
                    const std::vector<std::string> &imacNames,
                    const std::vector<GridCell> &fov,
                    const std::filesystem::path &inDir,
                    const int &numRuns = 10) {

  // Prepare output
  std::vector<std::vector<double>> results{};
  std::filesystem::path outFile{
      "../../data/results/prelim_exps/checkpoint_test/results.csv"};
  int timeBound{33};

  // Iterate through models
  for (int i{0}; i < imacs.size(); ++i) {
    std::cout << "MODEL:" << imacNames.at(i) << "\n";

    // Get run files, fixedIMacExecutor, and ground truth IMac model
    std::shared_ptr<FixedIMacExecutor> exec{
        getExecutor(inDir, "five_semi_static", std::make_pair(5, 5), numRuns)};

    // Get the robot object
    std::shared_ptr<CoverageRobot> robot{std::make_shared<POMDPCoverageRobot>(
        GridCell{0, 0}, timeBound, 5, 5, fov, exec, imacs.at(i))};

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

int main() {

  // Robot FOV
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{0, -1}, GridCell{1, -1},
                            GridCell{-1, 0},  GridCell{1, 0},  GridCell{-1, 1},
                            GridCell{0, 1},   GridCell{1, 1}};

  // IMac directory
  std::filesystem::path inDir{"../../data/prelim_exps"};

  // Number of runs
  int numRuns{10};

  // Read in the iMac models
  std::vector<std::shared_ptr<IMac>> imacs{};
  std::vector<std::string> imacNames{};

  std::vector<std::string> learningTypes{"posterior_sampling",
                                         "maximum_likelihood"};
  std::vector<int> checkpoints{0, 1, 5, 10, 50, 150};

  for (const std::string &type : learningTypes) {
    for (const int &checkpoint : checkpoints) {
      std::filesystem::path imacDir{"../../data/prelim_exps/checkpoints"};
      imacDir /= type;
      imacDir /= ("episode_" + std::to_string(checkpoint));

      imacs.push_back(std::make_shared<IMac>(imacDir));
      imacNames.push_back(type + "_episode_" + std::to_string(checkpoint));
    }
  }
  // Add ground truth
  imacs.push_back(
      std::make_shared<IMac>("../../data/prelim_exps/five_semi_static"));
  imacNames.push_back("ground_truth");

  // RUn the experiments
  runExperiments(imacs, imacNames, fov, inDir, numRuns);
}