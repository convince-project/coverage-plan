/**
 * Implementation of the BIMac class in bimac.h.
 * @see bimac.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/bimac.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <filesystem>
#include <memory>

// TODO: _readBIMacMatrix

// TODO: _writeBIMacMatrix

// TODO: sample

/**
 * Compute the MLE given the observed data.
 *
 * This amounts to computing the mode of each Beta distribution.
 * mode = (alpha - 1) / (alpha + beta - 2)
 */
std::shared_ptr<IMac> BIMac::mle() {
  Eigen::MatrixXd entryMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaEntry.rows(), this->_alphaEntry.cols(), [&](Eigen::Index i) {
        return (this->_alphaEntry(i) - 1.0) /
               (this->_alphaEntry(i) + this->_betaEntry(i) - 2.0);
      })};

  Eigen::MatrixXd exitMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaExit.rows(), this->_alphaExit.cols(), [&](Eigen::Index i) {
        return (this->_alphaExit(i) - 1.0) /
               (this->_alphaExit(i) + this->_betaExit(i) - 2.0);
      })};

  // TODO: Avoid repition here, perhaps using a helper function
  // TODO: When alpha and beta are 1, return 0.5 in matrix?
  // TODO: Create a more complex lambda expression which checks the (1,1)
  // condition

  return std::make_shared<IMac>(entryMatrix, exitMatrix);
}

/**
 * Updates the BIMac posterior given a new set of observations.
 */
void BIMac::updatePosterior(std::vector<BIMacObservation> observations) {

  for (BIMacObservation obs : observations) {
    // Update lambda_entry parameters
    this->_alphaEntry(obs.x, obs.y) += obs.freeToOccupied;
    this->_betaEntry(obs.x, obs.y) += obs.freeToFree;

    // Update lambda exit parameters
    this->_alphaExit(obs.x, obs.y) += obs.occupiedToFree;
    this->_betaExit(obs.x, obs.y) += obs.occupiedToOccupied;
  }
}

/**
 * Writes each of the BIMac matrices out to file
 */
void BIMac::writeBIMac(std::filesystem::path outDir) {
  // Each matrix is stored in a different file
  _writeBIMacMatrix(this->_alphaEntry, outDir / "alpha_entry.txt");
  _writeBIMacMatrix(this->_betaEntry, outDir / "beta_entry.txt");
  _writeBIMacMatrix(this->_alphaExit, outDir / "alpha_exit.txt");
  _writeBIMacMatrix(this->_betaExit, outDir / "beta_exit.txt");
}