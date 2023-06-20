/**
 * Implementation of the CoverageRobot class in coverage_robot.h
 * @see coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_robot.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <fstream>
#include <memory>
#include <vector>

/**
 * Return the IMac instance to be used for an episode.
 */
std::shared_ptr<IMac> CoverageRobot::_getIMacInstanceForEpisode() {
  return this->_bimac->posteriorSample();
}

/**
 * Wrapper around _planFn which fills in the gaps from class members.
 */
Action
CoverageRobot::planNextAction(int time, std::shared_ptr<IMac> imac,
                              const std::vector<IMacObservation> &obsVector) {
  return this->_planFn(this->_currentLoc, time, this->_timeBound, imac,
                       this->_covered, obsVector);
}

/**
 * Wrapper around _executeFn which fills in the gaps from class members.
 */
ActionOutcome CoverageRobot::executeAction(const Action &action) {
  return this->_executeFn(this->_currentLoc, action);
}

/**
 * Wrapper around _observeFn which fills in the gaps from class members.
 */
std::vector<IMacObservation> CoverageRobot::makeObservations() {
  return this->_observeFn(this->_currentLoc);
}

/**
 * Resets all necessary members for the next episode.
 */
void CoverageRobot::resetForNextEpisode(const GridCell &startLoc,
                                        int timeBound) {
  this->_covered.clear();
  this->_currentLoc = startLoc;
  this->_timeBound = timeBound;
}

/**
 * Logs a set of covered nodes to file.
 */
void CoverageRobot::logCoveredLocations(const std::filesystem::path &outFile) {
  std::ofstream f(outFile);
  if (f.is_open()) {
    for (GridCell &cell : this->_covered) {
      f << cell.x << ',' << cell.y << '\n';
    }
    f.close();
  }
}

// TODO: Implement CoverageRobot::runCoverageEpisode