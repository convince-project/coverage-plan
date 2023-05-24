/**
 * Implementation of the IMacExecutor class in imac_executor.h.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <memory>
#include <random>
#include <tuple>
#include <vector>
/**
 * Samples a state from a matrix of bernoulli random variables
 */
Eigen::MatrixXd IMacExecutor::_sampleState(Eigen::MatrixXd distMatrix) {

  // Generate a uniform rng between 0 and 1
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<double> sampler{0.0, 1.0};

  // Return 1 if sampled double is <= value in distMatrix, else 0
  // A value of 1 means the cell is occupied
  return Eigen::MatrixXd::NullaryExpr(
      distMatrix.rows(), distMatrix.cols(), [&](Eigen::Index i) {
        return (sampler(gen) <= distMatrix(i)) ? 1.0 : 0.0;
      });
}

/**
 * Restarts the MoD execution
 */
Eigen::MatrixXd IMacExecutor::restart() {
  return this->_sampleState(this->_imac->computeInitialBelief());
}

/**
 * Updates the current MoD state based on the IMac model and observations
 */
Eigen::MatrixXd
IMacExecutor::updateState(std::vector<std::tuple<int, int, int>> observations) {
  // First, sample through the IMac model
  Eigen::MatrixXd nextState{this->_imac->forwardStep(this->_currentState)};

  // Explicitly set the values in the observation list
  for (std::tuple<int, int, int> obs : observations) {
    nextState(std::get<0>(obs), std::get<1>(obs)) = std::get<2>(obs);
  }

  return nextState;
}