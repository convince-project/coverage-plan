/**
 * Header file for the IMacExecutor class.
 *
 * IMacExecutor is used to sample runs through an IMac model.
 *
 * @author Charlie Street
 */
#ifndef IMAC_EXECUTOR_H
#define IMAC_EXECUTOR_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/util/seed.h"
#include <Eigen/Dense>
#include <filesystem>
#include <memory>
#include <random>
#include <vector>

/**
 * Struct for storing IMac observations.
 *
 * Note an observation at (x,y) corresponds to element (y,x) in the matrices.
 * This is so we map to the Cartesian coordinates the robot operates over.
 *
 * Members:
 * cell: The cell on the grid
 * occupied: 1 if cell is occupied, otherwise 0
 */
struct IMacObservation {
  GridCell cell{};
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
  std::uniform_real_distribution<double> _sampler{};
  std::vector<Eigen::MatrixXi> _mapDynamics{};

  /**
   * Store the current state of the map for logging purposes.
   */
  void _addMapForTs();

protected:
  // Used for random sampling alongside _sampler
  std::mt19937_64 _gen{};

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
      : _imac{imac}, _currentState{}, _gen{SeedHelpers::genRandomDeviceSeed()},
        _sampler{0.0, 1.0}, _mapDynamics{} {}

  /**
   * Restart the simulation and return the new initial state.
   *
   * @param observations A vector of IMacObservations (what is seen at t=0).
   * Optional.
   *
   * @return initialState the initial state of the map of dynamics
   */
  Eigen::MatrixXi restart(const std::vector<IMacObservation> &observations =
                              std::vector<IMacObservation>{});

  /**
   * Update the current MoD state based on the iMac model, where successor
   * values are constrained to match the observations we have made.
   *
   * @param observations A vector of IMacObservations
   *
   * @return nextState The successor IMac state
   */
  Eigen::MatrixXi updateState(const std::vector<IMacObservation> &observations);

  /**
   * Output the map dynamic information into a csv file.
   *
   * Each row is the state of the map at the corresponding timestep.
   * Row format: ts,(x,y,occ)*
   *
   * @param outFile The CSV file to write the map logs
   */
  void logMapDynamics(const std::filesystem::path &outFile);
};

#endif