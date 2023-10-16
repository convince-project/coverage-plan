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
    runFiles.push_back(inDir / ("episode_" + std::to_string(r) + ".csv"));
  }
  return std::make_shared<FixedIMacExecutor>(runFiles, dim.first, dim.second);
}

/**
 * Compute the error between an estimated IMac and a ground truth IMac.
 *
 * The error is computed as the sum of absolute errors in parameters.
 * This function has been copied from bimac_learning_experiment.cpp
 *
 * @param estimate The estimated IMac instance
 * @param groundTruth The ground truth IMac instance
 *
 * @return error The sum of absolute errors
 */
double computeError(std::shared_ptr<IMac> estimate,
                    std::shared_ptr<IMac> groundTruth) {

  double error{0};

  Eigen::MatrixXd estimateEntry{estimate->getEntryMatrix()};
  Eigen::MatrixXd groundTruthEntry{groundTruth->getEntryMatrix()};
  Eigen::MatrixXd estimateExit{estimate->getExitMatrix()};
  Eigen::MatrixXd groundTruthExit{groundTruth->getExitMatrix()};
  Eigen::MatrixXd estimateInit{estimate->getInitialBelief()};
  Eigen::MatrixXd groundTruthInit{groundTruth->getInitialBelief()};

  for (int i{0}; i < estimateEntry.rows(); ++i) {
    for (int j{0}; j < estimateEntry.cols(); ++j) {
      error += abs(estimateInit(i, j) - groundTruthInit(i, j));

      // If a part of the Markov chain isn't reachable, ignore it
      if (!(groundTruthExit(i, j) == 0.0 && groundTruthInit(i, j) == 1.0)) {
        error += abs(estimateEntry(i, j) - groundTruthEntry(i, j));
      }

      if (!(groundTruthEntry(i, j) == 0.0 && groundTruthInit(i, j) == 0.0)) {
        error += abs(estimateExit(i, j) - groundTruthExit(i, j));
      }
    }
  }

  return error;
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

void runGroundTruth() {
  std::filesystem::path imacDir{"../../data/prelim_exps/lifelong_test"};
  std::shared_ptr<IMac> groundTruthImac{std::make_shared<IMac>(imacDir)};

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
    std::vector<double> imacErrors{};

    // Start from scratch for each method
    // Episodes will be played in same order
    std::shared_ptr<FixedIMacExecutor> exec{
        getExecutor(imacDir, std::make_pair(5, 5), numEpisodes)};

    std::unique_ptr<POMDPCoverageRobot> robot{
        std::make_unique<POMDPCoverageRobot>(initPos, timeBound, 5, 5, fov,
                                             exec, groundTruthImac)};

    // Get initial error

    for (int episode{1}; episode <= numEpisodes; ++episode) {
      std::cout << "Method: Ground Truth; Episode: " << episode << '\n';

      // Write output logs to dummy file
      results.push_back(
          robot->runCoverageEpisode("/tmp/episodeVisited.csv").propCovered);
    }
    writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                          "ground_truth_results.csv");
  }
}

void runDifferentEstimates() {
  std::filesystem::path imacDir{"../../data/prelim_exps/lifelong_test"};
  std::shared_ptr<IMac> groundTruthImac{std::make_shared<IMac>(imacDir)};

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
    std::vector<double> imacErrors{};

    // Start from scratch for each method
    // Episodes will be played in same order
    std::shared_ptr<FixedIMacExecutor> exec{
        getExecutor(imacDir, std::make_pair(5, 5), numEpisodes)};

    std::unique_ptr<POMDPCoverageRobot> robot{
        std::make_unique<POMDPCoverageRobot>(initPos, timeBound, 5, 5, fov,
                                             exec, nullptr, method)};

    // Get initial error
    std::shared_ptr<IMac> estimate{robot->getBIMac()->mle()};
    imacErrors.push_back(computeError(estimate, groundTruthImac));

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

      // Get current iMac error using BiMac MLE estimate
      std::shared_ptr<IMac> estimate{robot->getBIMac()->mle()};
      imacErrors.push_back(computeError(estimate, groundTruthImac));
    }

    if (method == ParameterEstimate::posteriorSample) {
      writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                            "posterior_sample_results.csv");
      writeResults(imacErrors, "../../data/results/prelim_exps/lifelong_test/"
                               "posterior_sample_imac_errors.csv");
    } else if (method == ParameterEstimate::posteriorMean) {
      writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                            "posterior_mean_results.csv");
      writeResults(imacErrors, "../../data/results/prelim_exps/lifelong_test/"
                               "posterior_mean_imac_errors.csv");
    } else if (method == ParameterEstimate::maximumLikelihood) {
      writeResults(results, "../../data/results/prelim_exps/lifelong_test/"
                            "maximum_likelihood_results.csv");
      writeResults(imacErrors, "../../data/results/prelim_exps/lifelong_test/"
                               "maximum_likelihood_imac_errors.csv");
    }
  }
}

int main() {
  runGroundTruth();
  return 0;
}
