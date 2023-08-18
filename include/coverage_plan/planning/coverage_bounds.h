/**
 * Header file for custom bounds for coverage planning.
 *
 * @author Charlie Street
 */

#ifndef COVERAGE_BOUNDS_H
#define COVERAGE_BOUNDS_H

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/action.h"
#include <Eigen/Dense>
#include <despot/core/globals.h>
#include <despot/core/history.h>
#include <despot/interface/default_policy.h>
#include <despot/interface/lower_bound.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/upper_bound.h>
#include <despot/random_streams.h>
#include <memory>
#include <vector>

/**
 * An upper bound which returns the max cells that can still be covered.
 * This looks like min(num_cells-state.visited.size(),time_bound-state.time)
 * Attributes:
 *  _numCells: The number of cells in the map
 * _timeBound: The max time bound on planning
 */
class MaxCellsUpperBound : public despot::ParticleUpperBound {
private:
  const int _numCells{};
  const int _timeBound{};

public:
  /**
   * Constructor initialises attributes.
   *
   * @param numCells The number of cells in the map
   * @param timeBound The max time bound on planning
   */
  MaxCellsUpperBound(const int &numCells, const int &timeBound)
      : ParticleUpperBound{}, _numCells{numCells}, _timeBound{timeBound} {}

  /**
   * Returns an upper bound on the max reward obtainable from state.
   *
   * Note the coverage planning problem is finite, with no discount factor.
   *
   * @param state The state to evaluate the reward from
   * @return maxCumulativeReward The max cumulative reward obtainable from state
   */
  double Value(const despot::State &state) const;
};

/**
 * A very simple lower bound which just returns zero.
 * This is part of my effort to enforce a finite horizon within despot.
 */
class ZeroParticleLowerBound : public despot::ParticleLowerBound {
public:
  /**
   * Initialise model to nullptr as not used.
   */
  ZeroParticleLowerBound() : ParticleLowerBound(nullptr) {}

  /**
   * The action shouldn't matter here, just returning 0.0 as horizon reached.
   *
   * @param particles A vector of states
   *
   * @return valuedAction The corresponding action and value (0.0)
   */
  despot::ValuedAction
  Value(const std::vector<despot::State *> &particles) const;
};

/**
 * A default policy which greedily chooses action based on immediate reward.
 *
 * Attributes:
 * As in superclass, plus:
 * _ imacEntry: The IMac entry matrix
 * _imacExit: The IMac exit matrix
 *
 */
class GreedyCoverageDefaultPolicy : public despot::DefaultPolicy {

private:
  const Eigen::MatrixXd _imacEntry{};
  const Eigen::MatrixXd _imacExit{};
  const despot::ParticleLowerBound *particlePtrToDelete{};

public:
  /**
   * Constructor initialises attributes.
   *
   * @param model The POMDP
   * @param particleLowerBound A lower bound on the cumulative reward
   * @param imac An IMac instance
   */
  GreedyCoverageDefaultPolicy(const despot::DSPOMDP *model,
                              despot::ParticleLowerBound *particleLowerBound,
                              std::shared_ptr<IMac> imac)
      : DefaultPolicy{model, particleLowerBound},
        _imacEntry{imac->getEntryMatrix()}, _imacExit{imac->getExitMatrix()} {}

  /**
   * Function greedily chooses an action weighted on the particles.
   *
   * @param particles The states at the head (latest?) of the scenarios
   * @param streams Random streams attached to the scenarios
   * @param history The current action-observation history
   */
  despot::ACT_TYPE Action(const std::vector<despot::State *> &particles,
                          despot::RandomStreams &streams,
                          despot::History &history) const;
};

#endif