/**
 * Implementation of CoverageWorld in coverage_world.h.
 * @see coverage_world.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_world.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/coverage_state.h"
#include <despot/interface/pomdp.h>
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