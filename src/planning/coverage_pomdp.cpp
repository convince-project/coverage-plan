/**
 * Implementation of the CoveragePOMDP class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_state.h"
#include <despot/interface/pomdp.h>
#include <iostream>
#include <set>

// TODO: Step

/**
 * Returns the total number of actions.
 *
 * For coverage planning problems, its 5 (up, down, left, right, wait)
 */
int CoveragePOMDP::NumActions() const { return 5; }

/**
 * Return the probability of observing obs given we took action and reached
 * state.
 */
double CoveragePOMDP::ObsProb(despot::OBS_TYPE obs, const despot::State &state,
                              despot::ACT_TYPE action) const {

  // TODO: Get FOV in here somehow
  // Don't need action success marker here, just the obs vector
  std::vector<IMacObservation> obsVec{
      std::get<0>(Observation::fromObsType(obs, this->_fov))};

  const CoverageState &coverageState{static_cast<const CoverageState &>(state)};

  // If one cell doesn't match, we return 0.0. If all good, we return 1.0
  for (const IMacObservation &imacObs : obsVec) {

    // Recall grid cells in obsVec are relative to robot's pos
    int x{coverageState.robot_position.x + imacObs.cell.x};
    int y{coverageState.robot_position.y + imacObs.cell.y};

    if (x < 0 or x >= coverageState.map.cols() or y < 0 or
        y >= coverageState.map.rows()) {
      // Out of bounds location should always be marked as occupied
      if (!imacObs.occupied) {
        return 0.0;
      }
    } else {
      // coverageState.map has to be indexed (y,x)
      if (coverageState.map(y, x) != imacObs.occupied) {
        return 0.0;
      }
    }
  }

  return 1.0;
}

// TODO: InitialBelief

/**
 * Returns the maximal *immediate* reward.
 *
 * For coverage planning problems, it is 1.0
 * (awarded for reaching an uncovered location)
 */
double CoveragePOMDP::GetMaxReward() const { return 1.0; }

// TODO: GetBestAction

void CoveragePOMDP::PrintState(const despot::State &state,
                               std::ostream &out = std::cout) const {
  const CoverageState &coverageState{static_cast<const CoverageState &>(state)};
  // Convert to set for quicker lookup
  std::set<GridCell> coveredSet{};
  std::copy(coverageState.covered.begin(), coverageState.covered.end(),
            std::inserter(coveredSet, coveredSet.end()));

  // Print out timestep and coverage %
  out << "Time: " << coverageState.time;
  out << "; Coverage: "
      << ((double)coverageState.covered.size()) /
             ((double)coverageState.map.size())
      << "%\n";

  // Print out map using the flipping convention of the map
  for (int x{0}; x < coverageState.map.cols(); ++x) {
    for (int y{0}; y < coverageState.map.rows(); ++y) {
      // Print covered cells in green
      if (coveredSet.find(GridCell{x, y}) != coveredSet.end()) {
        out << "\032[1;31m";
      } else {
        out << "\033[1;0m";
      }

      // Robot is R, occupied is X, free is -
      if (coverageState.robot_position.x == x &&
          coverageState.robot_position.y == y) {
        out << "R ";
      } else if (coverageState.map(y, x) == 1) {
        out << "X ";
      } else {
        out << "- ";
      }
    }
    out << '\n';
  }
}

// TODO: PrintObs

/**
 * Prints an action.
 */
void CoveragePOMDP::PrintAction(despot::ACT_TYPE action,
                                std::ostream &out = std::cout) const {
  // Just writes the action name, nothing fancy here
  switch (ActionHelpers::fromInt(action)) {
  case Action::up:
    out << "Action: Up\n";
    break;
  case Action::down:
    out << "Action: Down\n";
    break;
  case Action::left:
    out << "Action: Left\n";
    break;
  case Action::right:
    out << "Action: Right\n";
    break;
  case Action::wait:
    out << "Action: Wait\n";
    break;
  default: // Should never be called given Action enum
    out << "\n";
  }
}

// TODO: PrintBelief

/**
 * Allocate a state using the memory pool.
 * This is really just copying what was in the DESPOT tutorial.
 */
despot::State *CoveragePOMDP::Allocate(int state_id = -1,
                                       double weight = 0) const {
  CoverageState *state = this->_memory_pool.Allocate();
  state->state_id = state_id;
  state->weight = weight;
  return state;
}

/**
 * Copy a state.
 * Copied with minor modifications from the DESPOT tutorial.
 */
despot::State *CoveragePOMDP::Copy(const despot::State *particle) const {
  CoverageState *state = this->_memory_pool.Allocate();
  *state = *static_cast<const CoverageState *>(particle);
  state->SetAllocated();
  return state;
}

/**
 * Deallocate a state in the memory pool.
 * Copied with minor modifications from the DESPOT tutorial.
 */
void CoveragePOMDP::Free(despot::State *state) const {
  this->_memory_pool.Free(static_cast<CoverageState *>(state));
}

/**
 * Returns the number of allocated particles (sampled states).
 * Follows examples in returning the number of particles in the memory pool.
 */
int CoveragePOMDP::NumActiveParticles() const {
  return this->_memory_pool.num_allocated();
}