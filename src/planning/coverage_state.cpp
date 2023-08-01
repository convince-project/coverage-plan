/**
 * Implementation of the CoverageState class in coverage_state.h
 * @see coverage_state.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_state.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

/**
 * Produces a string description of a state.
 *
 * @return stateStr The state as a string
 */
std::string CoverageState::text() const {

  std::ostringstream stream{};
  stream << std::setprecision(2);

  // Convert to set for quicker lookup
  std::set<GridCell> coveredSet{};
  std::copy(this->covered.begin(), this->covered.end(),
            std::inserter(coveredSet, coveredSet.end()));

  // Write out timestep and coverage %
  stream << "Time: " << this->time << "; Coverage: "
         << int(round(100 * ((double)this->covered.size()) /
                      ((double)this->map.size())))
         << "%\n";

  // Write out map using the flipping convention of the map
  for (int y{0}; y < this->map.rows(); ++y) {
    for (int x{0}; x < this->map.cols(); ++x) {
      // Write covered cells in green
      if (coveredSet.find(GridCell{x, y}) != coveredSet.end()) {
        stream << "\x1b[1;32m";
      } else {
        stream << "\033[1;0m";
      }

      // Robot is R, occupied is X, free is -
      if (this->robotPosition.x == x && this->robotPosition.y == y) {
        stream << "R ";
      } else if (this->map(y, x) == 1) {
        stream << "X ";
      } else {
        stream << "- ";
      }
    }
    stream << '\n';
  }
  stream << "\033[1;0m";
  return stream.str();
}