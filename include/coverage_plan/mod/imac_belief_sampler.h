/**
 * Extension of IMacExecutor which doesn't store a current state.
 * This class can effectively just sample from an IMac state, with the option
 * to set the seed, as required by DESPOT.
 *
 * @author Charlie Street
 */

#ifndef IMAC_BELIEF_SAMPLER_H
#define IMAC_BELIEF_SAMPLER_H

#include "coverage_plan/mod/imac_executor.h"

/**
 * Subclass which removes all functionality except IMac belief sampling.
 * This removes any function which uses this->_currentState (hence stateless).
 * The IMac model isn't even used, but  want to reuse the sampling
 * functionality. This is necessary for use within DEPSOT. When sampling, the
 * random seed can be specified.
 *
 * Members:
 * As in superclass
 */
class IMacBeliefSampler : public IMacExecutor {

public:
  /**
   * Initialises all attributes, where this->_imac is set to a nullptr.
   */
  IMacBeliefSampler() : IMacExecutor(nullptr) {}

  /**
   * Sample from a belief over the current IMac state.
   *
   * @param distMatrix The belief over the current IMac state.
   * @param seed The random seed (if 0, use current seed)
   * @param observations Any observations made at the current timestep
   *
   * @return sampledState The sampled IMac state
   */
  Eigen::MatrixXi
  sampleFromBelief(const Eigen::MatrixXd &distMatrix, const double &seed = 0.0,
                   const std::vector<IMacObservation> &observations =
                       std::vector<IMacObservation>{});

  /**
   * Function not implemented in IMacBeliefSampler.
   */
  Eigen::MatrixXi restart(const std::vector<IMacObservation> &observations =
                              std::vector<IMacObservation>{}) {
    throw "restart not implemented in IMacBeliefSampler.";
  }

  /**
   * Function not implemented in IMacBeliefSampler.
   */
  Eigen::MatrixXi
  updateState(const std::vector<IMacObservation> &observations) {
    throw "updateState not implemented in IMacBeliefSampler.";
  }

  /**
   * Function not implemented in IMacBeliefSampler.
   */
  void logMapDynamics(const std::filesystem::path &outFile) {
    throw "logMapDynamics not implemented in IMacBeliefSampler.";
  }

  /**
   * Function not implemented in IMacBeliefSampler.
   */
  void clearRobotPosition(const GridCell &cell) {
    throw "clearRobotPosition not implemented in IMacBeliefSampler.";
  }
};

#endif