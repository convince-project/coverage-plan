/**
 * Implementation of functions in IMacBeliefSampler.
 * @see imac_belief_sampler.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac_belief_sampler.h"
#include "coverage_plan/util/seed.h"

/**
 * Sample from a belief over the current IMac state.
 */
Eigen::MatrixXi IMacBeliefSampler::sampleFromBelief(
    const Eigen::MatrixXd &distMatrix, const double &seed,
    const std::vector<IMacObservation> &observations) {
  if (seed != 0.0) {
    this->_gen.seed(SeedHelpers::doubleToUInt64(seed));
  }
  Eigen::MatrixXi sampledState{this->_sampleState(distMatrix)};

  // Add the observations into the sampledState
  for (IMacObservation obs : observations) {
    // y is row, x is column
    sampledState(obs.cell.y, obs.cell.x) = obs.occupied;
  }

  return sampledState;
}