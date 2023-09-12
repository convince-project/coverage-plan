/**
 * Implementation of GreedyCoverageRobot in greedy_coverage_robot.h.
 * @see greedy_coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/baselines/greedy_coverage_robot.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/util/seed.h"
#include <algorithm>
#include <math.h>
#include <random>
#include <vector>

/**
 * Greedily selects action which maximises immediate reward.
 */
Action
GreedyCoverageRobot::_planFn(const GridCell &currentLoc,
                             const std::vector<Action> &enabledActions, int ts,
                             int timeBound, std::shared_ptr<IMac> imac,
                             const std::vector<GridCell> &visited,
                             const std::vector<IMacObservation> &currentObs) {

  // Roll current belief forward to get occupancy probs at next time step
  Eigen::MatrixXd nextBelief{imac->forwardStep(this->_belief->getMapBelief())};

  // bestAct stores all actions with the best reward
  std::vector<Action> bestAct{};
  double maxReward{0.0};

  // Get expected reward for each action (prob of it being free and unvisited)
  for (const Action &act : enabledActions) {
    GridCell nextLoc{ActionHelpers::applySuccessfulAction(currentLoc, act)};
    // If not out of bounds and not visited yet
    if ((!nextLoc.outOfBounds(0, nextBelief.cols(), 0, nextBelief.rows())) &&
        std::count(visited.begin(), visited.end(), nextLoc) == 0) {
      double reward{1.0 - nextBelief(nextLoc.y, nextLoc.x)};
      if (reward > maxReward) {
        maxReward = reward;
        bestAct.clear();
        bestAct.push_back(act);
      } else if (fabs(reward - maxReward) < 0.0001) {
        bestAct.push_back(act);
      }
    }
  }

  // If there are no non-visited actions, choose an enabled action at random
  if (bestAct.size() == 0) {
    bestAct = enabledActions;
  }

  // Sample one of the best actions at random
  std::mt19937_64 gen{SeedHelpers::genRandomDeviceSeed()};
  std::uniform_int_distribution<> sampler{0, (int)bestAct.size() - 1};

  return bestAct.at(sampler(gen));
}