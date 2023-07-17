/**
 * Implementation of functions in coverage-plan/planning/action.h
 * @see action.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/action.h"
#include "coverage_plan/mod/grid_cell.h"

/**
 * Applies a successful action to a grid cell to get the new location.
 */
GridCell applySuccessfulAction(const GridCell &cell, const Action &action) {
  switch (action) {
  case Action::up:
    return GridCell{cell.x, cell.y - 1};
  case Action::down:
    return GridCell{cell.x, cell.y + 1};
  case Action::left:
    return GridCell{cell.x - 1, cell.y};
  case Action::right:
    return GridCell{cell.x + 1, cell.y};
  case Action::wait:
    return cell;
  default: // Not really all that clear what to put here, so just cell
    return cell;
  }
}