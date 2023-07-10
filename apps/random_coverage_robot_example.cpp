/**
 * Example script which runs a random coverage robot.
 *
 * @author: Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/coverage_robot.h"
#include <Eigen/Dense>
#include <filesystem>
#include <memory>
#include <random>
#include <vector>

/*
 * Synthesises a random valid action for the coverage robot.
 * Recall that x goes from left to right, y from top to bottom.
 */
Action randomAction(const GridCell &currentLoc, int ts, int timeBound,
                    std::shared_ptr<IMac> imac,
                    const std::vector<GridCell> &covered,
                    const std::vector<IMacObservation> &obsVector) {

  std::vector<Action> validActions{Action::wait};

  // Used to check dimensions of map
  Eigen::MatrixXd entryMat{imac->getEntryMatrix()};

  // up: y-1
  if (currentLoc.y - 1 >= 0) {
    validActions.push_back(Action::up);
  }

  // down: y+1
  if (currentLoc.y + 1 < entryMat.rows()) {
    validActions.push_back(Action::down);
  }

  // left: x-1
  if (currentLoc.x - 1 >= 0) {
    validActions.push_back(Action::left);
  }

  // right: x+1
  if (currentLoc.x + 1 < entryMat.cols()) {
    validActions.push_back(Action::right);
  }

  std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<> sampler{0, validActions.size() - 1};

  return validActions.at(sampler(gen));
}

ActionOutcome execute(const Eigen::MatrixXi &nextState,
                      const GridCell &currentLoc, const Action &action) {
  GridCell nextLoc{};
  switch (action) {
  case Action::up:
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y - 1;
    break;
  case Action::down:
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y + 1;
    break;
  case Action::left:
    nextLoc.x = currentLoc.x - 1;
    nextLoc.y = currentLoc.y;
    break;
  case Action::right:
    nextLoc.x = currentLoc.x + 1;
    nextLoc.y = currentLoc.y;
    break;
  case Action::wait:
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y;
    break;
  }

  bool succ{true};
  // Check for action failure (note flipped x and y)
  if (nextState[nextLoc.y, nextLoc.x] == 1) {
    succ = false;
    nextLoc.x = currentLoc.x;
    nextLoc.y = currentLoc.y;
  }

  return ActionOutcome{action, succ, nextLoc};
}

/*
 * Not worried about receiving observations in this example.
 */
std::vector<IMacObservation> observe(const GridCell &currentLoc) {
  return std::vector<IMacObservation>{};
}

/*
 * Creates the robot, wrapping the execute function to handle the internals.
 */
std::shared_ptr<CoverageRobot> createRobot(const Eigen::MatrixXi &nextState) {

  // TODO: Check nextState is adjusted properly. If it isn't, have a pointer
  auto executeLambda{[&](const GridCell &currentLoc, const Action &action) {
    return execute(nextState, currentLoc, action);
  }};
  return std::make_shared<CoverageRobot>(GridCell{0, 0}, 100, 10, 10,
                                         randomAction, executeLambda, observe);
}

int main() {

  // This should probably be a pointer to an IMacExecutor instance.
  // Then we can actually do the iMacExecution in the execute function
  // passing it in via lambda capture
  // One of these function should then also do the logging...
  // Again, this should probably be handled in the executeFunction too.
  Eigen::MatrixXi currentMapState{};

  std::shared_ptr<CoverageRobot> coverageRobot{createRobot(currentMapState)};

  coverageRobot->runCoverageEpisode(
      "/home/charlie/work/coverage-plan/data/results/"
      "randomCoverageRobotRun.csv");

  return 0;
}
