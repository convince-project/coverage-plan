/**
 * Implementation of the CoverageRobot class in coverage_robot.h
 * @see coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_robot.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <vector>

/**
 * Return the IMac instance to be used for an episode.
 */
std::shared_ptr<IMac> CoverageRobot::_getIMacInstanceForEpisode() {
  if (this->_groundTruthIMac == nullptr) {
    return this->_bimac->posteriorSample();
  } else {
    return this->_groundTruthIMac;
  }
}

/**
 * Helper function to add an initial state observation to a BIMacObservation
 */
void CoverageRobot::_addInitObservation(
    std::map<GridCell, BIMacObservation> &biMacObsMap,
    const IMacObservation &obs) {

  if (!obs.cell.outOfBounds(0, this->_xDim, 0, this->_yDim)) {
    // count returns number of keys of obs.cell
    if (biMacObsMap.count(obs.cell) == 0) {
      biMacObsMap[obs.cell] = BIMacObservation{obs.cell, 0, 0, 0, 0, 0, 0};
    }
    if (obs.occupied == 1) {
      biMacObsMap[obs.cell].initOccupied += 1;
    } else {
      biMacObsMap[obs.cell].initFree += 1;
    }
  }
}

/**
 * Helper function to add IMac transition observation to a BIMacObservation.
 */
void CoverageRobot::_addTransitionObservation(
    std::map<GridCell, BIMacObservation> &biMacObsMap, const GridCell &cell,
    const int &prevState, const int &nextState) {

  // Have we got an observation struct for this cell?
  if (!cell.outOfBounds(0, this->_xDim, 0, this->_yDim)) {
    if (biMacObsMap.count(cell) == 0) {
      biMacObsMap[cell] = BIMacObservation{cell, 0, 0, 0, 0, 0, 0};
    }
    if (prevState == 0 && nextState == 0) {
      biMacObsMap[cell].freeToFree += 1;
    } else if (prevState == 0 && nextState == 1) {
      biMacObsMap[cell].freeToOccupied += 1;
    } else if (prevState == 1 && nextState == 0) {
      biMacObsMap[cell].occupiedToFree += 1;
    } else {
      biMacObsMap[cell].occupiedToOccupied += 1;
    }
  }
}

/**
 * Converts IMacObservation vectors into a BIMacObservation vector.
 */
std::vector<BIMacObservation> CoverageRobot::_generateBIMacObservations(
    const std::vector<std::vector<IMacObservation>> &observations) {

  std::map<GridCell, BIMacObservation> biMacObsMap{};
  std::map<GridCell, int> prevObsMap{};
  std::map<GridCell, int> currentObsMap{};

  // Deal with initial observation
  std::vector<IMacObservation> initObs{observations.at(0)};
  for (const IMacObservation &obs : initObs) {
    if (!obs.cell.outOfBounds(0, this->_xDim, 0, this->_yDim)) {
      prevObsMap[obs.cell] = obs.occupied;
      this->_addInitObservation(biMacObsMap, obs);
    }
  }

  // Iterate through each timestep
  for (int i{1}; i < observations.size(); ++i) {
    for (const IMacObservation &obs : observations.at(i)) {
      if (!obs.cell.outOfBounds(0, this->_xDim, 0, this->_yDim)) {
        currentObsMap[obs.cell] = obs.occupied;
      }
    }
    // Iterate over preObsMap
    for (const auto &elem : prevObsMap) {
      // Have we got a BIMac observation?
      if (currentObsMap.count(elem.first) > 0) {
        this->_addTransitionObservation(biMacObsMap, elem.first, elem.second,
                                        currentObsMap[elem.first]);
      }
    }
    // Prepare for next iteration
    prevObsMap = currentObsMap;
    currentObsMap.clear();
  }

  // Convert biMac map into biMac vector
  std::vector<BIMacObservation> biMacObsVector{};
  for (const auto &elem : biMacObsMap) {
    biMacObsVector.push_back(elem.second);
  }

  return biMacObsVector;
}

/**
 * Returns a vector of actions that can be executed from the current location.
 */
std::vector<Action> CoverageRobot::_getEnabledActions() {
  std::vector<Action> validActions{Action::wait};

  // up: y-1
  if (this->_currentLoc.y - 1 >= 0) {
    validActions.push_back(Action::up);
  }

  // down: y+1
  if (this->_currentLoc.y + 1 < this->_yDim) {
    validActions.push_back(Action::down);
  }

  // left: x-1
  if (this->_currentLoc.x - 1 >= 0) {
    validActions.push_back(Action::left);
  }

  // right: x+1
  if (this->_currentLoc.x + 1 < this->_xDim) {
    validActions.push_back(Action::right);
  }

  return validActions;
}

/**
 * Function for printing current transition to stdout.
 *
 */
void CoverageRobot::_printCurrentTransition(const GridCell &startLoc,
                                            const ActionOutcome &outcome) {

  // Bools written as strings
  std::cout << std::boolalpha;

  std::cout << "STATE: (" << startLoc.x << ',' << startLoc.y << "); ACTION: ";
  // Get action string
  switch (outcome.action) {
  case Action::up:
    std::cout << "up";
    break;
  case Action::down:
    std::cout << "down";
    break;
  case Action::left:
    std::cout << "left";
    break;
  case Action::right:
    std::cout << "right";
    break;
  case Action::wait:
    std::cout << "wait";
    break;
  }

  std::cout << "; SUCCESS: " << outcome.success << "; SUCCESSOR: (";
  std::cout << outcome.location.x << ',' << outcome.location.y << ")\n";
}

/**
 * Wrapper around _planFn which fills in the gaps from class members.
 */
Action
CoverageRobot::planNextAction(int time, std::shared_ptr<IMac> imac,
                              const std::vector<IMacObservation> &obsVector) {
  return this->_planFn(this->_currentLoc, this->_getEnabledActions(), time,
                       this->_timeBound, imac, this->_visited, obsVector);
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
  this->_visited.clear();
  this->_currentLoc = startLoc;
  this->_timeBound = timeBound;
}

/**
 * Logs a set of visited nodes to file.
 */
void CoverageRobot::logVisitedLocations(const std::filesystem::path &outFile) {
  std::ofstream f{outFile};
  if (f.is_open()) {
    for (const GridCell &cell : this->_visited) {
      f << cell.x << ',' << cell.y << '\n';
    }
    f.close();
  }
}

/**
 * Run the plan-execute-observe cycle for a single episode, up to _timeBound.
 */
void CoverageRobot::runCoverageEpisode(const std::filesystem::path &outFile) {

  // Current timestep
  int t{0};

  // Unique locations set for termination check
  std::set<GridCell> covered{};

  std::shared_ptr<IMac> imacForEpisode{this->_getIMacInstanceForEpisode()};
  int numCells{(int)(imacForEpisode->getEntryMatrix().rows() *
                     imacForEpisode->getEntryMatrix().cols())};

  // At each timestep, we get a vector of observations
  std::vector<std::vector<IMacObservation>> observations{};

  // Add initial location to visited and take initial observations
  this->_visited.push_back(this->_currentLoc);
  covered.insert(this->_currentLoc);
  observations.push_back(this->makeObservations());

  while (t < this->_timeBound and covered.size() < numCells) {

    Action nextAction{this->planNextAction(
        t, imacForEpisode, observations.at(observations.size() - 1))};

    ActionOutcome outcome{this->executeAction(nextAction)};

    // Add current observation to observations
    observations.push_back(this->makeObservations());

    // Update location, visited, covered, and time
    this->_currentLoc = outcome.location;
    this->_visited.push_back(this->_currentLoc);
    covered.insert(this->_currentLoc);
    ++t;
  }

  // Update BiMac
  this->_bimac->updatePosterior(this->_generateBIMacObservations(observations));

  // Log results
  this->logVisitedLocations(outFile);
}