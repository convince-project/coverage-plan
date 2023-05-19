/**
 * Implementation of the IMac class in imac.h.
 *
 * @author Charlie Street
 */

#include "coverage-plan/mod/imac.h"
#include <Eigen/Dense>
#include <memory>

/**
 * Returns initial belief over IMac map of dynamics.
 */
std::shared_ptr<Eigen::MatrixXd> IMac::computeInitialBelief() {
  // Check if already cached
  if (this->_initialBelief == nullptr) {
    this->_initialBelief{std::make_shared<Eigen::MatrixXd>(
        this->_exitMatrix.rows(), this->exitMatrix.cols())};

    // Initial belief assumes equal chance of being in each state previously
    *(this->_initialBelief) =
        (0.5 * this->_entryMatrix) + (0.5 * (1 - this->_exitMatrix));
  }

  return this->_initialBelief;
}

/**
 * Runs a given state through IMac to get distribution for the next timestep
 */
std::unique_ptr<Eigen::MatrixXd>
IMac::forwardStep(std::unique_ptr<Eigen::MatrixXd> currentState) {
  std::unique_ptr<Eigen::MatrixXd> nextState{std::make_unique<Eigen::MatrixXd>(
      this->_exitMatrix.rows(), this->exitMatrix.cols())};

  // Prob of switching to occupied from free, or staying occupied
  *nextState = ((1 - currentState) * this->_entryMatrix) +
               (currentState * (1 - this->_exitMatrix));

  return nextState;
}