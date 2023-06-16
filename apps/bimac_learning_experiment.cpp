/**
 * A small sanity check experiment demonstrating that BIMac learns.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/bimac.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

/**
 * Function creates a true IMac instance to learn.
 *
 * The model doesn't make much geometric sense, but there are:
 * - 20% static obstacles
 * - 30% static free space
 * - 25% rapidly changing space
 * - 25% semi-static obstacles
 *
 * @return groundTruth the ground truth IMac instance for the experiment
 */
std::shared_ptr<IMac> generateGroundTruthIMac() {
  Eigen::MatrixXd entryMatrix{10, 10};
  Eigen::MatrixXd exitMatrix{10, 10};
  Eigen::MatrixXd initialBelief{10, 10};

  int numSet{0};
  const int staticObsLimit{20};
  const int staticFreeLimit{50};
  const int semiStaticLimit{75};

  // Not concerned with coordinate systems here
  for (int i{0}; i < 10; ++i) {
    for (int j{0}; j < 10; ++j) {
      if (numSet < staticObsLimit) {
        entryMatrix(i, j) = 1.0;
        exitMatrix(i, j) = 0.0;
        initialBelief(i, j) = 1.0;
      } else if (numSet < staticFreeLimit) {
        entryMatrix(i, j) = 0.0;
        exitMatrix(i, j) = 1.0;
        initialBelief(i, j) = 0.0;
      } else if (numSet < semiStaticLimit) {
        entryMatrix(i, j) = 0.05;
        exitMatrix(i, j) = 0.05;
        initialBelief(i, j) = 0.3;
      } else {
        entryMatrix(i, j) = 0.5;
        exitMatrix(i, j) = 0.5;
        initialBelief(i, j) = 0.5;
      }
      ++numSet;
    }
  }

  return std::make_shared<IMac>(entryMatrix, exitMatrix, initialBelief);
}

/**
 * Compute the error between an estimated IMac and a ground truth IMac.
 *
 * The error is computed as the sum of absolute errors in parameters.
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
 * Run a single day of IMac (100 time steps).
 *
 * @param groundTruth The ground truth IMac instance
 * @param imacExec The imacExecutor
 * @param bimac The bimac instance
 *
 * @return errors A tuple containing (mle error, posterior mean error)
 */
std::tuple<double, double> runSingleDay(std::shared_ptr<IMac> groundTruth,
                                        std::shared_ptr<IMacExecutor> imacExec,
                                        std::shared_ptr<BIMac> bimac) {

  std::map<std::tuple<int, int>, BIMacObservation> obsMap;

  for (int i{0}; i < 10; ++i) {
    for (int j{0}; j < 10; ++j) {
      obsMap[std::make_tuple(i, j)] = BIMacObservation{i, j, 0, 0, 0, 0, 0, 0};
    }
  }

  Eigen::MatrixXi prevState{imacExec->restart()};
  Eigen::MatrixXi currentState{};

  for (int t{0}; t < 20; ++t) {
    currentState = imacExec->updateState(std::vector<IMacObservation>{});

    // Update observations
    for (int i{0}; i < 10; ++i) {
      for (int j{0}; j < 10; ++j) {
        int x{j};
        int y{i};
        if (prevState(i, j) == 0 && currentState(i, j) == 0) {
          obsMap[std::make_tuple(x, y)].freeToFree += 1;
        } else if (prevState(i, j) == 0 && currentState(i, j) == 1) {
          obsMap[std::make_tuple(x, y)].freeToOccupied += 1;
        } else if (prevState(i, j) == 1 && currentState(i, j) == 0) {
          obsMap[std::make_tuple(x, y)].occupiedToFree += 1;
        } else {
          obsMap[std::make_tuple(x, y)].occupiedToOccupied += 1;
        }

        if (t == 0) {
          if (prevState(i, j) == 0) {
            obsMap[std::make_tuple(x, y)].initFree += 1;
          } else {
            obsMap[std::make_tuple(x, y)].initOccupied += 1;
          }
        }
      }
    }

    prevState = currentState;
  }

  // Update BIMac
  std::vector<BIMacObservation> obsVector{};
  for (int i{0}; i < 10; ++i) {
    for (int j{0}; j < 10; ++j) {
      obsVector.push_back(obsMap[std::make_tuple(j, i)]);
    }
  }
  bimac->updatePosterior(obsVector);

  std::shared_ptr<IMac> mleIMac{bimac->mle()};
  std::shared_ptr<IMac> pmIMac{bimac->posteriorMean()};

  return std::make_tuple(computeError(mleIMac, groundTruth),
                         computeError(pmIMac, groundTruth));
}

/**
 * Function writes a given set of results for a method to file.
 *
 * @param label mle or pm for the respective methods
 * @param results A vector of results
 * @param outFile The file handle where we're storing the results
 */
void writeToFile(std::string_view label, std::vector<double> results,
                 std::ofstream &outFile) {

  // This part taken from:
  // https://stackoverflow.com/questions/20817322/
  // convert-vectorint-to-delimited-string
  std::ostringstream resultsToStr;
  if (!results.empty()) {
    std::copy(results.begin(), results.end() - 1,
              std::ostream_iterator<double>(resultsToStr, ","));
    resultsToStr << results.back();
  }

  // Writing to file
  outFile << label << "," << resultsToStr.str() << '\n';
}

int main() {
  std::shared_ptr<IMac> groundTruth{generateGroundTruthIMac()};
  std::shared_ptr<IMacExecutor> imacExec{
      std::make_shared<IMacExecutor>(groundTruth)};

  std::ofstream outFile{"/home/charlie/work/coverage-plan/data/results/"
                        "BIMacLearningExperimentResults.csv"};
  if (outFile.is_open()) {
    std::shared_ptr<BIMac> bimac{};
    for (int repeat{0}; repeat < 40; ++repeat) {
      // Start repeat with new BIMac instance
      bimac = std::make_shared<BIMac>(10, 10);

      std::vector<double> mleError{};
      std::vector<double> pmError{};

      // Compute initial error
      std::shared_ptr<IMac> mleIMac{bimac->mle()};
      std::shared_ptr<IMac> pmIMac{bimac->posteriorMean()};
      mleError.push_back(computeError(mleIMac, groundTruth));
      pmError.push_back(computeError(pmIMac, groundTruth));

      for (int day{0}; day < 400; ++day) {
        std::cout << "REPEAT: " << repeat + 1 << ", DAY: " << day + 1 << "\n";
        std::tuple<double, double> dayError{
            runSingleDay(groundTruth, imacExec, bimac)};
        mleError.push_back(std::get<0>(dayError));
        pmError.push_back(std::get<1>(dayError));
      }

      writeToFile("mle", mleError, outFile);
      writeToFile("pm", pmError, outFile);
    }

    outFile.close();
  }

  return 0;
}