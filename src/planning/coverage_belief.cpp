/**
 * Implementation of the CoverageBelief class in  coverage_belief.h.
 * @see coverage_belief.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_belief.h"
#include <despot/interface/belief.h>
#include <iomanip>
#include <sstream>
#include <string>

// TODO: Sample

// TODO: Update

/**
 * Convert belief into a string.
 */
std::string CoverageBelief::text() const {
  std::ostringstream stream{};

  double pctCovered{(double)this->_covered.size() /
                    (double)this->_mapBelief.size()};

  // Write out pos, time, percentage covered
  stream << std::setprecision(2) << "Robot Position: ("
         << this->_robotPosition.x << ", " << this->_robotPosition.y
         << "); Time: " << this->_time << "; Percentage Covered: " << pctCovered
         << "; Map Belief: \n";

  // Write out the map belief to a string
  for (int x{0}; x < this->_mapBelief.cols(); ++x) {
    for (int y{0}; y < this->_mapBelief.rows(); ++y) {
      stream << this->_mapBelief(y, x) << " ";
    }
    stream << "\n";
  }

  return stream.str();
}

/**
 * Make a copy of the belief.
 */
despot::Belief *CoverageBelief::MakeCopy() const {
  // Note: Allocated with new, so make sure this is deallocated...
  return new CoverageBelief(this->_robotPosition, this->_time, this->_covered,
                            this->_mapBelief, this->_imac);
}