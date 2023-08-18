/**
 * Implementation of the bounds in coverage_bounds.h.
 * @see coverage_bounds.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_bounds.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_state.h"
#include <algorithm>
#include <despot/core/globals.h>
#include <despot/interface/default_policy.h>
#include <iterator>

/**
 * Returns an upper bound on the max reward obtainable from state.
 */
double MaxCellsUpperBound::Value(const despot::State &state) const {
  const CoverageState &coverState{static_cast<const CoverageState &>(state)};
  return std::min((double)(this->_numCells - coverState.covered.size()),
                  (double)(this->_timeBound - coverState.time));
}

/**
 * The action shouldn't matter here, just returning 0.0 as horizon reached.
 */
despot::ValuedAction ZeroParticleLowerBound::Value(
    const std::vector<despot::State *> &particles) const {
  return despot::ValuedAction(ActionHelpers::toInt(Action::up), 0.0);
}

/**
 * Function greedily chooses an action weighted on the particles.
 *
 * @param particles The states at the head (latest?) of the scenarios
 * @param streams Random streams attached to the scenarios
 * @param history The current action-observation history
 */
despot::ACT_TYPE GreedyCoverageDefaultPolicy::Action(
    const std::vector<despot::State *> &particles,
    despot::RandomStreams &streams, despot::History &history) const {
  // Entry for each action
  std::vector<double> immRewards{};
  for (int i{0}; i < this->model_->NumActions(); ++i) {
    immRewards.push_back(0);
  }

  for (despot::State *state : particles) { // Get weighted immediate reward
    CoverageState *coverState{static_cast<CoverageState *>(state)};
    for (int a{0}; a < this->model_->NumActions(); ++a) {
      GridCell succLoc{ActionHelpers::applySuccessfulAction(
          coverState->robotPosition, ActionHelpers::fromInt(a))};
      if (coverState->covered.count(succLoc) != 0 &&
          !succLoc.outOfBounds(
              0, this->_imacEntry.cols(), 0,
              this->_imacEntry.rows())) { // Not already covered and in bounds
        // prob of being free in next step weighted by particle weight
        if (coverState->map(succLoc.y, succLoc.x) == 1) { // occupied, use exit
          immRewards.at(a) +=
              (this->_imacExit(succLoc.y, succLoc.x) * coverState->weight);
        } else { // free, use 1 - entry
          immRewards.at(a) += ((1.0 - this->_imacEntry(succLoc.y, succLoc.x)) *
                               coverState->weight);
        }
      }
    }
  }
  // Get location of max element (max action)
  return std::distance(immRewards.begin(),
                       std::max_element(immRewards.begin(), immRewards.end()));
}