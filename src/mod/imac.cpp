/**
 * Implementation of the IMac class in imac.h.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <memory>

/**
 * Returns initial belief over IMac map of dynamics.
 */
Eigen::MatrixXd IMac::computeInitialBelief() {
  // Check if already cached - if it isn't, the matrix will be empty
  if (this->_initialBelief.size() == 0) {
    Eigen::MatrixXd ones{Eigen::MatrixXd::Ones(this->_entryMatrix.rows(),
                                               this->_entryMatrix.cols())};
    this->_initialBelief =
        (0.5 * this->_entryMatrix) + (0.5 * (ones - this->_exitMatrix));
  }
  return this->_initialBelief;
}

/**
 * Runs a given belief or state through IMac to get distribution for the next
 * timestep
 */
Eigen::MatrixXd IMac::forwardStep(Eigen::MatrixXd currentBelief) {
  Eigen::MatrixXd ones{Eigen::MatrixXd::Ones(this->_entryMatrix.rows(),
                                             this->_entryMatrix.cols())};
  return (ones - currentBelief).cwiseProduct(this->_entryMatrix) +
         currentBelief.cwiseProduct((ones - this->_exitMatrix));
}