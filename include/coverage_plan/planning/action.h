/**
 * Header file for action-based structures and functions.
 *
 * @author Charlie Street
 */

#ifndef ACTION_H
#define ACTION_H

#include "coverage_plan/mod/grid_cell.h"

/**
 * Enum for robot actions on a grid.
 */
enum class Action { up, down, left, right, wait };

/**
 * Applies a successful action to a grid cell to get the new location.
 *
 * @param cell The current grid cell
 * @param action The action to be executed
 *
 * @return nextCell The new location
 */
GridCell applySuccessfulAction(const GridCell &cell, const Action &action);

/**
 * Struct for storing action outcomes.
 *
 * Action outcome contains action success/failure flag, the successor
 * location, and a reminder of the action executed.
 *
 * Members:
 * action: The action executed
 * success: A flag set to true if the action was successful
 * location: The current GridCell of the robot
 */
struct ActionOutcome {
  Action action{};
  bool success{};
  GridCell location{};
};

#endif