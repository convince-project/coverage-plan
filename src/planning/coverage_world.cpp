/**
 * Implementation of CoverageWorld in coverage_world.h.
 * @see coverage_world.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_world.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_state.h"
#include <despot/interface/pomdp.h>
#include <set>
#include <vector>

/**
 * No 'connection' needed here, just returns true.
 */
bool CoverageWorld::Connect() { return true; }

/**
 * Returns the current CoverageState of the system.
 */
despot::State *CoverageWorld::GetCurrentState() const { return this->state_; }

/**
 * Resets the IMacExecutor and returns the initial coverage state.
 */
despot::State *CoverageWorld::Initialize() {
  // Cast so I can access members of CoverageState
  CoverageState *coverState{static_cast<CoverageState *>(this->state_)};

  // These aren't important, but good to set
  coverState->weight = 1.0;
  coverState->state_id = -1;

  coverState->robotPosition = this->_initPos;
  coverState->time = this->_initTime;
  coverState->covered = std::vector<GridCell>{this->_initPos};

  // Make sure to set the robot's initial position to free :)
  coverState->map = this->_exec->restart(
      std::vector<IMacObservation>{IMacObservation{this->_initPos, 0}});

  return coverState;
}

/**
 * Print a state.
 */
void CoverageWorld::PrintState(const despot::State &s, ostream &out) const {
  out << s.text();
}

/**
 * Execute action, update state, and make observation.
 */
bool CoverageWorld::ExecuteAction(despot::ACT_TYPE action,
                                  despot::OBS_TYPE &obs) {

  CoverageState *coverState{static_cast<CoverageState *>(this->state_)};

  // Update map (no observations here)
  coverState->map = this->_exec->updateState(std::vector<IMacObservation>{});

  // Update time
  ++coverState->time;

  // Update robot position
  GridCell succLoc{ActionHelpers::applySuccessfulAction(
      coverState->robotPosition, ActionHelpers::fromInt(action))};

  ActionOutcome outcome{};
  outcome.action = ActionHelpers::fromInt(action);
  outcome.success = true;

  if ((succLoc.outOfBounds(0, coverState->map.cols(), 0,
                           coverState->map.rows()) ||
       (coverState->map(succLoc.y, succLoc.x) == 1)) &&
      ActionHelpers::fromInt(action) != Action::wait) {
    outcome.success = false;
    succLoc = coverState->robotPosition;
  }

  outcome.location = succLoc;
  coverState->robotPosition = succLoc;

  // Update robot pos in map
  coverState->map = this->_exec->clearRobotPosition(coverState->robotPosition);

  // Update covered
  coverState->covered.push_back(coverState->robotPosition);

  // Compute observation
  obs = Observation::computeObservation(
      coverState->map, coverState->robotPosition, outcome, this->_fov);

  // Number of cells covered
  std::set<GridCell> uniqueCovered{};
  for (const GridCell &cell : coverState->covered) {
    uniqueCovered.insert(cell);
  }

  // Termination condition (time bound reached or all cells covered)
  if (coverState->time >= this->_timeBound or
      uniqueCovered.size() == coverState->map.size()) {
    return true;
  }
  return false;
}