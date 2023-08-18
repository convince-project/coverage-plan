/**
 * Header file for the CoveragePOMDP class.
 *
 * This class captures the simulated POMDP model used for coverage planning.
 *
 * CoveragePOMDP implements the DSPOMDP abstract base class in DESPOT.
 *
 * @author Charlie Street
 */
#ifndef COVERAGE_POMDP_H
#define COVERAGE_POMDP_H

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_belief_sampler.h"
#include "coverage_plan/planning/coverage_bounds.h"
#include "coverage_plan/planning/coverage_state.h"
#include <despot/interface/default_policy.h>
#include <despot/interface/lower_bound.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/upper_bound.h>
#include <despot/util/memorypool.h>
#include <memory>
#include <string>
#include <vector>

/**
 * POMDP model for coverage planning problems under spatio temporal uncertainty.
 *
 * Members:
 * _memory_pool: A memory pool for allocating CoverageStates
 * _fov: A vector of GridCells relative to the robot's position which capture
 * its FOV.
 * _imac: The IMac instance used for planning
 * _beliefSampler: The IMac belief sampler used for the simulator
 * _timeBound: The planning horizon in timesteps
 */
class CoveragePOMDP : public despot::DSPOMDP {
private:
  mutable despot::MemoryPool<CoverageState> _memoryPool{};
  const std::vector<GridCell> _fov{};
  std::shared_ptr<IMac> _imac{};
  std::unique_ptr<IMacBeliefSampler> _beliefSampler{};
  const int _timeBound{};

public:
  /**
   * Constructor initialises members.
   *
   * @param fov The robot's field of view as a vector of grid cells relative
   * to the robot's position
   * @param imac The IMac instance used for planning
   * @param timeBound The planning horizon in timesteps
   */
  CoveragePOMDP(const std::vector<GridCell> &fov, std::shared_ptr<IMac> imac,
                int timeBound)
      : despot::DSPOMDP{}, _memoryPool{}, _fov{fov}, _imac{imac},
        _timeBound{timeBound}, _beliefSampler{
                                   std::make_unique<IMacBeliefSampler>()} {}

  /**
   * The deterministic simulative model for the POMDP.
   * Avoids enumerating the whole POMDP, which is huge.
   *
   * @param state The current state of the system (assumed known). Should be
   * updated with the successor state
   * @param random_num A single random number which should dictate the
   * successor state and observation
   * @param action The action to be executed
   * @param reward The reward value. Should be updated with current step's
   * reward
   * @param obs The current observation. Should be updated with current
   * step's observation
   *
   * @return terminal True if the successor state is a terminal state
   */
  bool Step(despot::State &state, double random_num, despot::ACT_TYPE action,
            double &reward, despot::OBS_TYPE &obs) const;

  /**
   * Returns the number of actions.
   * Makes the assumption that all actions are enabled at each state (not
   * ideal).
   *
   * TODO: In the future, see if I can extend this to be dependent on the state
   * of belief
   *
   * @return numActions The number of executable actions
   */
  int NumActions() const;

  /**
   * Return the probability of observing obs given we took action and reached
   * state.
   *
   * @param obs The latest observation
   * @param state The state *we ended up at* after executing action
   * @param action The action executed
   *
   * @return obsProb The probability of making the observation
   */
  double ObsProb(despot::OBS_TYPE obs, const despot::State &state,
                 despot::ACT_TYPE action) const;

  /**
   * Return the initial belief, which corresponds to the initial IMac belief.
   *
   * @param start A (partial) initial state, e.g. the robot's initial position
   * @param type A parameter which allows to specify the type of the belief
   *
   * @return initBelief The initial belief
   */
  despot::Belief *InitialBelief(const despot::State *start,
                                std::string type = "DEFAULT") const;

  /**
   * Returns the maximal *immediate* reward.
   *
   * @return maxReward The maximum immediate reward
   */
  double GetMaxReward() const;

  /**
   * Returns (a, v), where a is an action with largest minimum reward when it is
   * executed, and v is its minimum reward, that is, a = \max_{a'} \min_{s}
   * R(a', s), and v = \min_{s} R(a, s) (copied from superclass as I can't
   * explain it better).
   *
   * @return bestAction The action with the largest minimum reward
   */
  despot::ValuedAction GetBestAction() const;

  /**
   * Override to allow for MaxCellsUpperBound.
   * @param name 				  Name of the upper bound
   * @param particleBoundName Name of the base ParticleUpperBound
   *
   * @return upperBound The upper bound
   */
  despot::ScenarioUpperBound *
  CreateScenarioUpperBound(std::string name = "DEFAULT",
                           std::string particleBoundName = "DEFAULT") const;

  /**
   * Override to allow for ZeroParticleLowerBound.
   *
   * @param name Name of the particle lower bound
   *
   * @return pLowerBound The particle lower bound
   */
  despot::ParticleLowerBound *
  CreateParticleLowerBound(std::string name = "DEFAULT") const;

  /**
   * Override to allow for GreedyCoverageDefaultPolicy.
   * @param name 				  Name of the lower bound
   * @param particleBoundName Name of the ParticleLowerBound to be used
   */
  despot::ScenarioLowerBound *
  CreateScenarioLowerBound(std::string bound_name = "DEFAULT",
                           std::string particleBoundName = "DEFAULT") const;

  /**
   * Prints a state.
   *
   * @param state The state to print
   * @param out The stream to write the state to
   */
  void PrintState(const despot::State &state,
                  std::ostream &out = std::cout) const;

  /**
   * Prints an observation.
   *
   * @param state The current state
   * @param obs The latest observation
   * @param out The stream to write the observation to
   */
  void PrintObs(const despot::State &state, despot::OBS_TYPE obs,
                std::ostream &out = std::cout) const;

  /**
   * Prints an action.
   *
   * @param action The action to print
   * @param out The stream to write the action to
   */
  void PrintAction(despot::ACT_TYPE action,
                   std::ostream &out = std::cout) const;

  /**
   * Prints a belief.
   *
   * @param belief The belief to print
   * @param out The stream to write the belief to
   */
  void PrintBelief(const despot::Belief &belief,
                   std::ostream &out = std::cout) const;

  /**
   * Allocate a state using the memory pool.
   *
   * @param state_id The state's ID
   * @param weight   The weight of the allocated state
   *
   * @return state A pointer to the newly allocated state
   */
  despot::State *Allocate(int state_id = -1, double weight = 0) const;

  /**
   * Copy a state.
   *
   * @param state The state to copy
   *
   * @return copy A pointer to the state copy
   */
  despot::State *Copy(const despot::State *state) const;

  /**
   * Deallocate a state in the memory pool.
   *
   * @param state The state to deallocate
   */
  void Free(despot::State *state) const;

  /**
   * Returns the number of allocated particles (sampled states).
   * All examples set this to the number of particles in the memory pool.
   *
   * @return numParticles The number of active particles
   */
  int NumActiveParticles() const;
};
#endif