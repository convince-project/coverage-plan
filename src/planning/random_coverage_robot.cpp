/**
 * Implementation of the RandomCoverageRobot class in random_coverage_robot.h
 * @see random_coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/random_coverage_robot.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/util/seed.h"
#include <Eigen/Dense>
#include <iostream>
#include <random>

/**
 * Function for printing current transition to stdout.
 *
 */
void RandomCoverageRobot::_printCurrentTransition(
    const GridCell &startLoc, const ActionOutcome &outcome) {

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
 * Synthesises a random valid action for the coverage robot.
 * Recall that x goes from left to right, y from top to bottom.
 */
Action
RandomCoverageRobot::_planFn(const GridCell &currentLoc,
                             const std::vector<Action> &enabledActions, int ts,
                             int timeBound, std::shared_ptr<IMac> imac,
                             const std::vector<GridCell> &covered,
                             const std::vector<IMacObservation> &currentObs) {
  std::mt19937_64 gen{SeedHelpers::genRandomDeviceSeed()};
  std::uniform_int_distribution<> sampler{0, (int)enabledActions.size() - 1};

  return enabledActions.at(sampler(gen));
}

/**
 * Executes an action by checking against the IMacExecutor.
 * Just applies the action to the grid and checks the outcome.
 */
ActionOutcome RandomCoverageRobot::_executeFn(const GridCell &currentLoc,
                                              const Action &action) {
  // Apply action
  GridCell nextLoc{ActionHelpers::applySuccessfulAction(currentLoc, action)};

  // Unroll the next IMac state here so we can check occlusion
  Eigen::MatrixXi nextState{
      this->_world->updateState(std::vector<IMacObservation>{})};

  bool succ{true};
  // Check for action failure (note flipped x and y)
  // Action failure occurs if intended location is occupied or intended location
  // out of bounds
  if (nextLoc.outOfBounds(0, nextState.cols(), 0, nextState.rows()) ||
      (nextState(nextLoc.y, nextLoc.x) == 1 && action != Action::wait)) {
    succ = false;
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y;
  }

  ActionOutcome outcome{action, succ, nextLoc};
  this->_printCurrentTransition(currentLoc, outcome);

  return outcome;
}

/**
 * Dummy observation function which returns an empty vector.
 */
std::vector<IMacObservation>
RandomCoverageRobot::_observeFn(const GridCell &currentLoc) {
  return std::vector<IMacObservation>{};
}
