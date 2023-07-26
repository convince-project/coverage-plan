/**
 * Implementation of the CoverageState class in coverage_state.h
 * @see coverage_state.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_state.h"
#include <string>

/**
 * Produces a string description of a state.
 */
std::string CoverageState::text() const {
  // Note: Full covered node list and map omitted from this string for
  // readability - this information will be included in the print functions
  return "Robot Position: (" + std::to_string(this->robot_position.x) + ", " +
         std::to_string(this->robot_position.y) +
         "); Time: " + std::to_string(this->time) +
         "; Nodes Covered: " + std::to_string(this->covered.size());
}