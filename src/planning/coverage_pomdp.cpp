/**
 * Implementation of the CoveragePOMDP class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_state.h"
#include <Eigen/Dense>
#include <algorithm>
#include <despot/core/globals.h>
#include <despot/interface/default_policy.h>
#include <despot/interface/pomdp.h>
#include <iostream>
#include <map>
#include <set>

/**
 * The deterministic simulative model for the POMDP.
 */
bool CoveragePOMDP::Step(despot::State &state, double random_num,
                         despot::ACT_TYPE action, double &reward,
                         despot::OBS_TYPE &obs) const {
  CoverageState &coverageState = static_cast<CoverageState &>(state);

  // Update the map state (only stochastic element)
  coverageState.map = this->_beliefSampler->sampleFromBelief(
      this->_imac->forwardStep(coverageState.map.cast<double>()), random_num);

  // The time is increased in each transition
  ++coverageState.time;

  GridCell expectedLoc{ActionHelpers::applySuccessfulAction(
      coverageState.robotPosition, ActionHelpers::fromInt(action))};
  ActionOutcome outcome{};
  outcome.action = ActionHelpers::fromInt(action);

  // Action success
  // Occurs if location in bounds and 0
  if ((!expectedLoc.outOfBounds(0, coverageState.map.cols(), 0,
                                coverageState.map.rows())) &&
      coverageState.map(expectedLoc.y, expectedLoc.x) == 0) {
    coverageState.robotPosition = expectedLoc;
    outcome.success = true;

    // Get a reward if we reach an previously unreached state
    if (std::find(coverageState.covered.begin(), coverageState.covered.end(),
                  expectedLoc) == coverageState.covered.end()) {
      reward = 1.0;
    } else {
      reward = 0.0;
    }

  } else {
    // If action failed, the robot's old location must be free
    coverageState.map(coverageState.robotPosition.y,
                      coverageState.robotPosition.x) = 0;
    outcome.success = false;
    if (action == ActionHelpers::toInt(Action::wait)) { // wait always succeeds
      outcome.success = true;
    }

    reward = 0.0;
  }

  outcome.location = coverageState.robotPosition;

  // Add to covered
  coverageState.covered.push_back(coverageState.robotPosition);

  // Now get the observation
  std::vector<IMacObservation> obsVec{};
  for (const GridCell &cell : this->_fov) {
    GridCell obsLoc{coverageState.robotPosition.x + cell.x,
                    coverageState.robotPosition.y + cell.y};
    if (!obsLoc.outOfBounds(0, coverageState.map.cols(), 0,
                            coverageState.map.rows())) {
      obsVec.push_back(
          IMacObservation{obsLoc, coverageState.map(obsLoc.y, obsLoc.x)});
    } else { // Out of bounds cells are occupied
      obsVec.push_back(IMacObservation{obsLoc, 1});
    }
  }
  obs = Observation::toObsType(obsVec, outcome);

  std::set<GridCell> uniqueCovered{};
  for (const GridCell &cell : coverageState.covered) {
    uniqueCovered.insert(cell);
  }

  int numCells{(int)(coverageState.map.rows() * coverageState.map.cols())};

  // Termination condition (time bound reached or all cells covered)
  if (coverageState.time >= this->_timeBound or
      uniqueCovered.size() == numCells) {
    return true;
  }
  return false;
}

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

  // Don't need action success marker here, just the obs vector
  std::vector<IMacObservation> obsVec{
      std::get<0>(Observation::fromObsType(obs, this->_fov))};

  const CoverageState &coverageState{static_cast<const CoverageState &>(state)};

  // If one cell doesn't match, we return 0.0. If all good, we return 1.0
  for (const IMacObservation &imacObs : obsVec) {

    // Recall grid cells in obsVec are relative to robot's pos
    GridCell obsLoc{coverageState.robotPosition.x + imacObs.cell.x,
                    coverageState.robotPosition.y + imacObs.cell.y};

    if (obsLoc.outOfBounds(0, coverageState.map.cols(), 0,
                           coverageState.map.rows())) {
      // Out of bounds location should always be marked as occupied
      if (!imacObs.occupied) {
        return 0.0;
      }
    } else {
      // coverageState.map has to be indexed (y,x)
      if (coverageState.map(obsLoc.y, obsLoc.x) != imacObs.occupied) {
        return 0.0;
      }
    }
  }

  return 1.0;
}

/**
 * Return the initial belief, which corresponds to the initial IMac belief.
 */
despot::Belief *CoveragePOMDP::InitialBelief(const despot::State *start,
                                             std::string type) const {
  // Assume default is our CoverageBelief
  if (type == "DEFAULT" || type == "COVERAGE_BELIEF") {
    // This should be the true initial state of the system which we have
    // privileged access to
    const CoverageState *initState{static_cast<const CoverageState *>(start)};

    Eigen::MatrixXd initMapBelief{this->_imac->getInitialBelief()};
    // Add initial observation into initial belief
    // The robot should be able to make an initial observation before moving
    for (const GridCell &cell : this->_fov) {
      GridCell absCell{initState->robotPosition.x + cell.x,
                       initState->robotPosition.y + cell.y};

      if (!absCell.outOfBounds(0, initMapBelief.cols(), 0,
                               initMapBelief.rows())) {
        initMapBelief(absCell.y, absCell.x) =
            initState->map(absCell.y, absCell.x);
      }
    }

    return new CoverageBelief(this, initState->robotPosition, initState->time,
                              initState->covered, initMapBelief, this->_imac,
                              this->_fov);
  } else { // Not supporting anything else (for now)
    std::cerr << "[CoveragePOMDP::InitialBelief] Unsupported belief type: "
              << type << '\n';
    exit(1);
  }
}

/**
 * Returns the maximal *immediate* reward.
 *
 * For coverage planning problems, it is 1.0
 * (awarded for reaching an uncovered location)
 */
double CoveragePOMDP::GetMaxReward() const { return 1.0; }

/**
 * Returns action with largest minimum immediate reward.
 * For coverage problems, all actions have minimum immediate reward of 0.
 * Therefore, I'll just choose up.
 */
despot::ValuedAction CoveragePOMDP::GetBestAction() const {
  return despot::ValuedAction(ActionHelpers::toInt(Action::up), 0.0);
}

/**
 * Prints a state.
 */
void CoveragePOMDP::PrintState(const despot::State &state,
                               std::ostream &out) const {
  out << state.text();
}

/**
 * Prints an observation.
 */
void CoveragePOMDP::PrintObs(const despot::State &state, despot::OBS_TYPE obs,
                             std::ostream &out) const {
  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, this->_fov)};

  // Print action success info
  if (std::get<1>(obsInfo)) {
    out << "Action Successful; Observation:\n";
  } else {
    out << "Action Failed; Observation:\n";
  }

  const CoverageState &coverageState{static_cast<const CoverageState &>(state)};
  int minX{(int)coverageState.map.cols()};
  int maxX{-1 * (int)coverageState.map.cols()};
  int minY{(int)coverageState.map.rows()};
  int maxY{-1 * (int)coverageState.map.rows()};

  std::map<GridCell, bool> obsMap{};

  // Get the range we have to print over
  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    obsMap[imacObs.cell] = (imacObs.occupied == 1);
    if (imacObs.cell.x < minX) {
      minX = imacObs.cell.x;
    }
    if (imacObs.cell.x > maxX) {
      maxX = imacObs.cell.x;
    }

    if (imacObs.cell.y < minY) {
      minY = imacObs.cell.y;
    }
    if (imacObs.cell.y > maxY) {
      maxY = imacObs.cell.y;
    }
  }

  // Print the observation as a grid
  for (int x{minX}; x <= maxX; ++x) {
    for (int y{minY}; y <= maxY; ++y) {
      if (x == 0 && y == 0) {
        out << "R "; // The robot
      } else if (obsMap.find(GridCell{x, y}) != obsMap.end()) {
        if (obsMap[GridCell{x, y}]) {
          out << "X "; // Occupied
        } else {
          out << "- "; // Free
        }
      } else {
        out << "? "; // Unobserved cells
      }
    }
    out << "\n";
  }
}

/**
 * Prints an action.
 */
void CoveragePOMDP::PrintAction(despot::ACT_TYPE action,
                                std::ostream &out) const {
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

/**
 * Prints a belief.
 */
void CoveragePOMDP::PrintBelief(const despot::Belief &belief,
                                std::ostream &out) const {
  out << belief.text();
}

/**
 * Allocate a state using the memory pool.
 * This is really just copying what was in the DESPOT tutorial.
 */
despot::State *CoveragePOMDP::Allocate(int state_id, double weight) const {
  CoverageState *state = this->_memoryPool.Allocate();
  state->state_id = state_id;
  state->weight = weight;
  return state;
}

/**
 * Copy a state.
 * Copied with minor modifications from the DESPOT tutorial.
 */
despot::State *CoveragePOMDP::Copy(const despot::State *particle) const {
  CoverageState *state = this->_memoryPool.Allocate();
  *state = *static_cast<const CoverageState *>(particle);
  state->SetAllocated();
  return state;
}

/**
 * Deallocate a state in the memory pool.
 * Copied with minor modifications from the DESPOT tutorial.
 */
void CoveragePOMDP::Free(despot::State *state) const {
  this->_memoryPool.Free(static_cast<CoverageState *>(state));
}

/**
 * Returns the number of allocated particles (sampled states).
 * Follows examples in returning the number of particles in the memory pool.
 */
int CoveragePOMDP::NumActiveParticles() const {
  return this->_memoryPool.num_allocated();
}