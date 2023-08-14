/**
 * Implementation of the RandomCoverageRobot class in random_coverage_robot.h
 * @see random_coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/random_coverage_robot.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/util/seed.h"
#include <Eigen/Dense>
#include <random>

/**
 * Synthesises a random valid action for the coverage robot.
 * Recall that x goes from left to right, y from top to bottom.
 */
Action
RandomCoverageRobot::_planFn(const GridCell &currentLoc,
                             const std::vector<Action> &enabledActions, int ts,
                             int timeBound, std::shared_ptr<IMac> imac,
                             const std::vector<GridCell> &visited,
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

  // Apply action in CoverageWorld
  despot::OBS_TYPE obs{0};
  this->_world->ExecuteAction(ActionHelpers::toInt(action), obs);

  // Get the action outcome object out of the observation
  bool succ{std::get<1>(Observation::fromObsType(obs, this->_fov))};
  GridCell nextLoc{currentLoc};
  if (succ) {
    nextLoc = ActionHelpers::applySuccessfulAction(currentLoc, action);
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

/**
 * Runs necessary setup for an episode.
 */
void RandomCoverageRobot::_episodeSetup(const GridCell &startLoc, const int &ts,
                                        const int &timeBound,
                                        std::shared_ptr<IMac> imacForEpisode) {
  CoverageRobot::_episodeSetup(startLoc, ts, timeBound, imacForEpisode);
  this->_world->Connect();
  this->_world->Initialize();
}