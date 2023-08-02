/**
 * Implementation of functions in coverage-plan/planning/action.h
 * @see action.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/action.h"
#include "coverage_plan/mod/grid_cell.h"

// Implementing functions in ActionHelpers namespace
namespace ActionHelpers {
/**
 * Applies a successful action to a grid cell to get the new location.
 * Note successful means not blocked or not out of bounds.
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

/**
 * Converts an action into an int.
 */
int toInt(const Action &action) {
  switch (action) {
  case Action::up:
    return 0;
  case Action::down:
    return 1;
  case Action::left:
    return 2;
  case Action::right:
    return 3;
  case Action::wait:
    return 4;
  default: // Can never happen, but will shut compiler up
    return -1;
  }
}

/**
 * Converts an int into an action.
 */
Action fromInt(const int &num) {
  switch (num) {
  case 0:
    return Action::up;
  case 1:
    return Action::down;
  case 2:
    return Action::left;
  case 3:
    return Action::right;
  case 4:
    return Action::wait;
  default:
    throw "Actions can only take integers in [0-4]";
  }
}
} // namespace ActionHelpers