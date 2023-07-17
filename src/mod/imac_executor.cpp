/**
 * Implementation of the IMacExecutor class in imac_executor.h.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <vector>

/**
 * Store the current state of the map for logging purposes.
 */
void IMacExecutor::_addMapForTs() {
  this->_mapDynamics.push_back(this->_currentState);
}

/**
 * Samples a state from a matrix of bernoulli random variables
 */
Eigen::MatrixXi IMacExecutor::_sampleState(const Eigen::MatrixXd &distMatrix) {
  // Return 1 if sampled double is <= value in distMatrix, else 0
  // A value of 1 means the cell is occupied
  return Eigen::MatrixXi::NullaryExpr(
      distMatrix.rows(), distMatrix.cols(), [&](Eigen::Index i) {
        return (this->_sampler(this->_gen) <= distMatrix(i)) ? 1 : 0;
      });
}

/**
 * Restarts the MoD execution
 */
Eigen::MatrixXi
IMacExecutor::restart(const std::vector<IMacObservation> &observations) {
  this->_mapDynamics.clear(); // Clear as new run
  this->_currentState = this->_sampleState(this->_imac->getInitialBelief());

  // Explicitly set the values in the observation list
  for (IMacObservation obs : observations) {
    // To make the matrix compatible with Cartesian coordinates,
    // y is the row number (y moves down), and x is the column (origin at (0,0))
    this->_currentState(obs.cell.y, obs.cell.x) = obs.occupied;
  }

  this->_addMapForTs();
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
    // y is row, x is column
    this->_currentState(obs.cell.y, obs.cell.x) = obs.occupied;
  }

  this->_addMapForTs();
  return this->_currentState;
}

/**
 * Output the map dynamic information into a csv file.
 */
void IMacExecutor::logMapDynamics(const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    int ts{0};
    for (const Eigen::MatrixXi &mapAtTs : this->_mapDynamics) {
      f << ts << ',';
      // Recall that coordinate (x,y) is (y,x) wrt. matrix indexing
      for (int y{0}; y < mapAtTs.rows(); ++y) {
        for (int x{0}; x < mapAtTs.cols(); ++x) {
          f << x << ',' << y << ',' << mapAtTs(y, x) << ',';
        }
      }
      f << '\n';
      ++ts;
    }
  }
  f.close();
}