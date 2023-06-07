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
#include <vector>
/**
 * Samples a state from a matrix of bernoulli random variables
 */
Eigen::MatrixXi IMacExecutor::_sampleState(const Eigen::MatrixXd &distMatrix) {

  // Generate a uniform rng between 0 and 1
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<double> sampler{0.0, 1.0};

  // Return 1 if sampled double is <= value in distMatrix, else 0
  // A value of 1 means the cell is occupied
  return Eigen::MatrixXi::NullaryExpr(
      distMatrix.rows(), distMatrix.cols(),
      [&](Eigen::Index i) { return (sampler(gen) <= distMatrix(i)) ? 1 : 0; });
}

/**
 * Restarts the MoD execution
 */
Eigen::MatrixXi IMacExecutor::restart() {
  this->_currentState = this->_sampleState(this->_imac->computeInitialBelief());
  return this->_currentState;
}

/**
 * Updates the current MoD state based on the IMac model and observations
 */
Eigen::MatrixXi
IMacExecutor::updateState(const std::vector<IMacObservation> &observations) {
  // First, sample through the next belief in the iMac model
  this->_currentState = this->_sampleState(
      _imac->forwardStep(this->_currentState.cast<double>()));

  // Explicitly set the values in the observation list
  for (IMacObservation obs : observations) {
    this->_currentState(obs.x, obs.y) = obs.occupied;
  }

  return this->_currentState;
}