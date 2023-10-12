/**
 * Small experiments to test the effect of numScenarios in DESPOT.
 * @author Charlie Street
 */

#include "coverage_plan/baselines/greedy_coverage_robot.h"
#include "coverage_plan/baselines/random_coverage_robot.h"
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
 * Get the output file for a particular method.
 *
 * @param outDir The output directory
 * @param numScenarios The number of scenarios used in DESPOT.
 *
 * @return outFile The results output file
 */
std::filesystem::path getOutputFile(const std::filesystem::path &outDir,
                                    const int &numScenarios) {

  return outDir /
         ("DESPOT_" + std::to_string(numScenarios) + "_scenarios_results.csv");
}

/**
 * Get the x,y dimensions of the map.
 *
 * @param env The name of the environment
 *
 * @return dim A pair of the x and y dimensions
 */
std::pair<int, int> getDimensions(const std::string &env) {
  if (env == "four_light" || env == "four_heavy") {
    return std::make_pair(4, 4);
  } else if (env == "five_light" || env == "five_heavy") {
    return std::make_pair(5, 5);
  }
  return std::make_pair(0, 0);
}

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
 * Create the coverage robot for an experiment.
 *
 * @param numScenarios The number of DESPOT scenarios to be sampled
 * @param timeBound The time bound for planning
 * @param fov The robot's fov
 * @param exec The FixedIMacExecutor
 * @param dim The x,y dimensions of the map
 * @param groundTruthIMac The ground truth IMac
 *
 * @return robot The coverage robot
 */
std::shared_ptr<CoverageRobot> getRobot(
    const int &numScenarios, const int &timeBound,
    const std::vector<GridCell> &fov, std::shared_ptr<FixedIMacExecutor> &exec,
    const std::pair<int, int> &dim, std::shared_ptr<IMac> groundTruthIMac) {

  return std::make_shared<POMDPCoverageRobot>(
      GridCell{0, 0}, timeBound, dim.first, dim.second, fov, exec,
      groundTruthIMac, ParameterEstimate::posteriorSample, "DEFAULT", 0.1,
      numScenarios);
}

/**
 * Write out the results for a method.
 *
 * @param results The results for each environment for a given method
 * @param envs The list of environments. Order matches results
 * @param outFile The file to write out to
 */
void writeResults(const std::vector<std::vector<CoverageResult>> &results,
                  const std::vector<std::string> &envs,
                  const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    for (int i{0}; i < results.size(); ++i) {
      f << envs.at(i) << ',';
      for (const CoverageResult &result : results.at(i)) {
        f << result.endTime << ',' << result.propCovered << ',';
      }
      f << '\n';
    }
  }
  f.close();
}

/**
 * Runs the experiments for different methods and different environments.
 *
 * @param scenarios The numScenarios values to evaluate
 * @param envs A list of environment types to read from file
 * @param timeBounds A vector of time bounds for different environments
 * @param fov The robot's field of view
 * @param inDir The directory to find all of the environment runs/IMac
 * models
 * @param outDir Where to store the results files
 * @param numRuns How many repeats are we running
 */
void runExperiments(const std::vector<int> &scenarios,
                    const std::vector<std::string> &envs,
                    const std::vector<int> &timeBounds,
                    const std::vector<GridCell> &fov,
                    const std::filesystem::path &inDir,
                    const std::filesystem::path &outDir,
                    const int &numRuns = 10) {

  // Iterate through numScenarios candidates
  for (const int &numScenarios : scenarios) {
    std::cout << "NUM SCENARIOS: " << numScenarios << "\n";

    // Prepare output
    std::vector<std::vector<CoverageResult>> resultsForMethod{};
    std::filesystem::path outFile{getOutputFile(outDir, numScenarios)};

    for (int envNum{0}; envNum < envs.size(); ++envNum) {
      // Get run files, fixedIMacExecutor, and ground truth IMac model
      std::pair<int, int> dim{getDimensions(envs.at(envNum))};
      std::shared_ptr<FixedIMacExecutor> exec{
          getExecutor(inDir, envs.at(envNum), dim, numRuns)};
      std::shared_ptr<IMac> groundTruthIMac{
          std::make_shared<IMac>(inDir / envs.at(envNum))};

      // Get the robot object
      std::shared_ptr<CoverageRobot> robot{
          getRobot(numScenarios, timeBounds.at(envNum), fov, exec, dim,
                   groundTruthIMac)};

      std::vector<CoverageResult> resultsForEnv{};
      for (int r{0}; r < numRuns; ++r) {
        std::cout << "ENVIRONMENT: " << envs.at(envNum) << ", RUN: " << r + 1
                  << "/" << numRuns << "\n";
        resultsForEnv.push_back(robot->runCoverageEpisode("/tmp/dummy.csv"));
      }
      resultsForMethod.push_back(resultsForEnv);
    }

    // Output file
    std::cout << "WRITING RESULTS\n";
    writeResults(resultsForMethod, envs, outFile);
  }
}

int main() {

  // Create numScenarios values to test
  std::vector<int> scenarios{100, 500};

  // Environment setup
  std::vector<std::string> envs{"four_light", "four_heavy", "five_light",
                                "five_heavy"};
  std::vector<int> timeBounds{25, 25, 40, 40};

  // Robot FOV
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{0, -1}, GridCell{1, -1},
                            GridCell{-1, 0},  GridCell{1, 0},  GridCell{-1, 1},
                            GridCell{0, 1},   GridCell{1, 1}};

  // IMac directory
  std::filesystem::path inDir{"../../data/prelim_exps"};

  // Output directory
  std::filesystem::path outDir{"../../data/results/num_scenarios_tuning"};

  // Number of runs
  int numRuns{10};

  // RUn the experiments
  runExperiments(scenarios, envs, timeBounds, fov, inDir, outDir, numRuns);
}