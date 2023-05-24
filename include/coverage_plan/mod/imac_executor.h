/**
 * Header file for the IMacExecutor class.
 *
 * IMacExecutor is used to sample runs through an IMac model.
 *
 * @author Charlie Street
 */
#ifndef IMAC_EXECUTOR_H
#define IMAC_EXECUTOR_H

#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <memory>
#include <tuple>
#include <vector>

/**
 * An executor class for an IMac model.
 * @see mod/imac.h
 * This class retains an IMac model and samples from it to generate runs of
 * the dynamics.
 */
class IMacExecutor {
private:
  std::shared_ptr<IMac> _imac{};
  Eigen::MatrixXd _currentState{};

  /**
   * Helper function which samples an MoD state from a distribution matrix.
   *
   * @param distMatrix a matrix of probabilities. Each probability is the
   * occupation probability for the current timestep.
   *
   * @return sampledState an MOD state sampled from the distribution
   */
  Eigen::MatrixXd _sampleState(Eigen::MatrixXd distMatrix);

public:
  /**
   * Constructor initialises the member variables.
   *
   * @param imac The IMac model
   */
  IMacExecutor(std::shared_ptr<IMac> imac) : _imac{imac}, _currentState{} {}

  /**
   * Restart the simulation and return the new initial state.
   *
   * @return initialState the initial state of the map of dynamics
   */
  Eigen::MatrixXd restart();

  /**
   * Update the current MoD state based on the iMac model, where successor
   * values are constrained to match the observations we have made.
   *
   * @param observations A vector of <x,y,occ_value (0 or 1)> values
   * representing the occupancy at grid cell (x,y)
   */
  Eigen::MatrixXd
  updateState(std::vector<std::tuple<int, int, int>> observations);
};

#endif