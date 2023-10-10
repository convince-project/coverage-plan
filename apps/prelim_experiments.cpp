/**
 * Preliminary experiments to optimise DESPOT parameters against baselines.
 * @author Charlie Street
 */

#include "coverage_plan/baselines/boustrophedon_coverage_robot.h"
#include "coverage_plan/baselines/energy_functional_coverage_robot.h"
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
 * @param pruningConstant The DESPOT pruning constant. -1 if not DESPOT method.
 * @param boundType The DESPOT bound types. Method name if not DESPOT method.
 *
 * @return outFile The results output file
 */
std::filesystem::path getOutputFile(const std::filesystem::path &outDir,
                                    const double &pruningConstant,
                                    const std::string &boundType) {
  if (pruningConstant < 0.0) {
    return outDir / (boundType + "_results.csv");
  } else {
    return outDir / ("DESPOT_pruning_" + std::to_string(pruningConstant) +
                     "_bound_" + boundType + "_results.csv");
  }
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
 * @param pruningConstant The DESPOT pruning constant. -1 if not DESPOT method.
 * @param boundType The DESPOT bound types. Method name if not DESPOT method.
 * @param timeBound The time bound for planning
 * @param fov The robot's fov
 * @param exec The FixedIMacExecutor
 * @param dim The x,y dimensions of the map
 * @param groundTruthIMac The ground truth IMac
 *
 * @return robot The coverage robot
 */
std::shared_ptr<CoverageRobot>
getRobot(const double &pruningConstant, const std::string &boundType,
         const int &timeBound, const std::vector<GridCell> &fov,
         std::shared_ptr<FixedIMacExecutor> &exec,
         const std::pair<int, int> &dim,
         std::shared_ptr<IMac> groundTruthIMac) {
  std::shared_ptr<CoverageRobot> robot{nullptr};

  if (pruningConstant < 0.0) {
    if (boundType == "RANDOM") {
      std::shared_ptr<CoverageWorld> world{std::make_shared<CoverageWorld>(
          GridCell{0, 0}, 0, timeBound, fov, exec)};
      robot = std::make_shared<RandomCoverageRobot>(
          GridCell{0, 0}, timeBound, dim.first, dim.second, world, fov,
          groundTruthIMac);
    } else if (boundType == "GREEDY") {
      robot = std::make_shared<GreedyCoverageRobot>(GridCell{0, 0}, timeBound,
                                                    dim.first, dim.second, fov,
                                                    exec, groundTruthIMac);
    } else if (boundType == "ENERGY_FUNCTIONAL") {
      robot = std::make_shared<EnergyFunctionalCoverageRobot>(
          GridCell{0, 0}, timeBound, dim.first, dim.second, fov, exec,
          groundTruthIMac);
    } else if (boundType == "BOUSTROPHEDON") {
      robot = std::make_shared<BoustrophedonCoverageRobot>(
          GridCell{0, 0}, timeBound, dim.first, dim.second, fov, exec,
          groundTruthIMac);
    }
  } else { // POMDP Coverage Robot
    robot = std::make_shared<POMDPCoverageRobot>(
        GridCell{0, 0}, timeBound, dim.first, dim.second, fov, exec,
        groundTruthIMac, boundType, pruningConstant);
  }

  return robot;
}

/**
 * Write out the results for a method.
 *
 * @param results The results for each environment for a given method
 * @param envs The list of environments. Order matches results
 * @param outFile The file to write out to
 */
void writeResults(const std::vector<std::vector<double>> &results,
                  const std::vector<std::string> &envs,
                  const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    for (int i{0}; i < results.size(); ++i) {
      f << envs.at(i) << ',';
      for (const double &propCovered : results.at(i)) {
        f << propCovered << ',';
      }
      f << '\n';
    }
  }
  f.close();
}

/**
 * Runs the experiments for different methods and different environments.
 *
 * @param methods A list of methods as (pruningConstant, boundType) pairs
 * @param envs A list of environment types to read from file
 * @param timeBounds A vector of time bounds for different environments
 * @param fov The robot's field of view
 * @param inDir The directory to find all of the environment runs/IMac
 * models
 * @param outDir Where to store the results files
 * @param numRuns How many repeats are we running
 */
void runExperiments(const std::vector<std::pair<double, std::string>> &methods,
                    const std::vector<std::string> &envs,
                    const std::vector<int> &timeBounds,
                    const std::vector<GridCell> &fov,
                    const std::filesystem::path &inDir,
                    const std::filesystem::path &outDir,
                    const int &numRuns = 10) {

  // Iterate through methods
  for (const std::pair<double, std::string> &method : methods) {
    double pruningConstant{std::get<0>(method)};
    std::string boundType{std::get<1>(method)};
    std::cout << "METHOD: PRUNING CONSTANT: " << pruningConstant
              << "; BOUND TYPE: " << boundType << "\n";

    // Prepare output
    std::vector<std::vector<double>> resultsForMethod{};
    std::filesystem::path outFile{
        getOutputFile(outDir, pruningConstant, boundType)};

    for (int envNum{0}; envNum < envs.size(); ++envNum) {
      // Get run files, fixedIMacExecutor, and ground truth IMac model
      std::pair<int, int> dim{getDimensions(envs.at(envNum))};
      std::shared_ptr<FixedIMacExecutor> exec{
          getExecutor(inDir, envs.at(envNum), dim, numRuns)};
      std::shared_ptr<IMac> groundTruthIMac{
          std::make_shared<IMac>(inDir / envs.at(envNum))};

      // Get the robot object
      std::shared_ptr<CoverageRobot> robot{
          getRobot(pruningConstant, boundType, timeBounds.at(envNum), fov, exec,
                   dim, groundTruthIMac)};

      std::vector<double> resultsForEnv{};
      for (int r{0}; r < numRuns; ++r) {
        std::cout << "ENVIRONMENT: " << envs.at(envNum) << ", RUN: " << r + 1
                  << "/" << numRuns << "\n";
        resultsForEnv.push_back(
            robot->runCoverageEpisode("/tmp/dummy.csv").propCovered);
      }
      resultsForMethod.push_back(resultsForEnv);
    }

    // Output file
    std::cout << "WRITING RESULTS\n";
    writeResults(resultsForMethod, envs, outFile);
  }
}

int main() {

  // Create methods to test
  std::vector<std::pair<double, std::string>> methods{
      std::make_pair(-1, "ENERGY_FUNCTIONAL"),
      std::make_pair(-1, "BOUSTROPHEDON")};
  // std::vector<std::pair<double, std::string>> methods{
  //    std::make_pair(-1, "RANDOM"), std::make_pair(-1, "GREEDY"),
  //    std::make_pair(-1, "ENERGY_FUNCTIONAL"),
  //    std::make_pair(-1, "BOUSTROPHEDON")};
  // std::vector<double> prunes{0.0, 0.01, 0.1, 1, 10};
  // std::vector<std::string> boundTypes{"TRIVIAL", "DEFAULT"};
  // for (const double &prune : prunes) {
  //  for (const std::string &type : boundTypes) {
  //    methods.push_back(std::make_pair(prune, type));
  //  }
  //}

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
  std::filesystem::path outDir{"../../data/results/prelim_exps"};

  // Number of runs
  int numRuns{10};

  // RUn the experiments
  runExperiments(methods, envs, timeBounds, fov, inDir, outDir, numRuns);
}