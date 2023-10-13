/**
 * Implementation of BoustrophedonCoverageRobot in
 * boustrophedon_coverage_robot.h.
 * @see boustrophedon_coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/baselines/boustrophedon_coverage_robot.h"
#include "coverage_plan/planning/action.h"
#include <algorithm>
#include <map>
#include <memory>
#include <vector>

/**
 * Follows a boustrophedon motion online.
 */
Action BoustrophedonCoverageRobot::_planFn(
    const GridCell &currentLoc, const std::vector<Action> &enabledActions,
    int ts, int timeBound, std::shared_ptr<IMac> imac,
    const std::vector<GridCell> &visited,
    const std::vector<IMacObservation> &currentObs) {

  // Get hash map of neighbouring observations
  std::map<GridCell, int> obsMap{};
  for (const IMacObservation &imacObs : currentObs) {
    obsMap[imacObs.cell] = imacObs.occupied;
  }

  // Priority of actions: up,down,right,left,wait
  std::vector<Action> actionPriority{Action::up, Action::down, Action::left,
                                     Action::right};

  for (const Action &action : actionPriority) {
    // Is the successor within the bounds of the map?
    bool inBounds{
        std::count(enabledActions.begin(), enabledActions.end(), action) > 0};
    if (!inBounds) {
      continue;
    }

    GridCell nextLoc{ActionHelpers::applySuccessfulAction(currentLoc, action)};

    // If we haven't observed all 4 neighbours we can't really do anything here.
    if (obsMap.count(nextLoc) == 0) {
      throw "[BoustrophedonCoverageRobot] Neighbouring direction not "
            "observed\n";
    }
    // Is the successor location *currently* free
    bool freeSpace{obsMap[nextLoc] == 0};

    // Have we covered the successor location?
    bool notCovered{std::count(visited.begin(), visited.end(), nextLoc) == 0};

    if (freeSpace && notCovered) {
      return action;
    }
  }

  return Action::wait;
}