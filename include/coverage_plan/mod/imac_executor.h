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
#include <random>
#include <vector>

/**
 * Struct for storing IMac observations.
 *
 * Members:
 * x: X position on grid
 * y: Y position on grid
 * occupied: 1 if cell is occupied, otherwise 0
 */
struct IMacObservation {
  int x{};
  int y{};
  int occupied{};
};

/**
 * An executor class for an IMac model.
 * @see mod/imac.h
 * This class retains an IMac model and samples from it to generate runs of
 * the dynamics.
 *
 * Members:
 * _imac: A shared ptr to an IMac model
 * _currentState: The current MoD state (deterministic, i.e. just 0s and 1s)
 *
 */
class IMacExecutor {
private:
  std::shared_ptr<IMac> _imac{};
  Eigen::MatrixXi _currentState{};
  // Used for random sampling - create once at start of class
  std::mt19937 _gen{};
  std::uniform_real_distribution<double> _sampler{};

  /**
   * Helper function which samples an MoD state from a distribution matrix.
   *
   * @param distMatrix a matrix of probabilities. Each probability is the
   * occupation probability for the current timestep.
   *
   * @return sampledState an MOD state sampled from the distribution
   */
  Eigen::MatrixXi _sampleState(const Eigen::MatrixXd &distMatrix);

public:
  /**
   * Constructor initialises the member variables.
   *
   * @param imac The IMac model
   */
  IMacExecutor(std::shared_ptr<IMac> imac)
      : _imac{imac}, _currentState{}, _gen{std::random_device{}()}, _sampler{
                                                                        0.0,
                                                                        1.0} {}

  /**
   * Restart the simulation and return the new initial state.
   *
   * @return initialState the initial state of the map of dynamics
   */
  Eigen::MatrixXi restart();

  /**
   * Update the current MoD state based on the iMac model, where successor
   * values are constrained to match the observations we have made.
   *
   * @param observations A vector of IMacObservations
   *
   * @return nextState The successor IMac state
   */
  Eigen::MatrixXi updateState(const std::vector<IMacObservation> &observations);
};

#endif