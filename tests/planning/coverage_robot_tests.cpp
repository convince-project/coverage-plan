/**
 * Tests for the CoverageRobot class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_robot.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <filesystem>
#include <fstream>
#include <vector>

// Dummy subclasses of CoverageRobot so we can test it
class TestCoverageRobotOne : public CoverageRobot {

private:
  Action _planFn(const GridCell &currentLoc,
                 const std::vector<Action> &enabledActions, int ts,
                 int timeBound, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &covered,
                 const std::vector<IMacObservation> &currentObs) {
    return Action::up;
  }

  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action) {
    return ActionOutcome{action, true, currentLoc};
  }

  std::vector<IMacObservation> _observeFn(const GridCell &currentLoc) {
    return std::vector<IMacObservation>{IMacObservation{currentLoc, 1}};
  }

public:
  TestCoverageRobotOne(const GridCell &currentLoc, int timeBound, int xDim,
                       int yDim)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim} {}
};

class TestCoverageRobotTwo : public CoverageRobot {

private:
  Action _planFn(const GridCell &currentLoc,
                 const std::vector<Action> &enabledActions, int ts,
                 int timeBound, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &covered,
                 const std::vector<IMacObservation> &currentObs) {
    return Action::up;
  }

  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action) {
    return ActionOutcome{action, true,
                         GridCell{currentLoc.x, currentLoc.y + 1}};
  }

  std::vector<IMacObservation> _observeFn(const GridCell &currentLoc) {
    return std::vector<IMacObservation>{IMacObservation{currentLoc, 1}};
  }

public:
  TestCoverageRobotTwo(const GridCell &currentLoc, int timeBound, int xDim,
                       int yDim)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim} {}
};

class TestCoverageRobotThree : public CoverageRobot {

private:
  Action _planFn(const GridCell &currentLoc,
                 const std::vector<Action> &enabledActions, int ts,
                 int timeBound, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &covered,
                 const std::vector<IMacObservation> &currentObs) {
    return Action::up;
  }

  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action) {
    return ActionOutcome{action, true,
                         GridCell{currentLoc.x, currentLoc.y + 1}};
  }

  std::vector<IMacObservation> _observeFn(const GridCell &currentLoc) {
    return std::vector<IMacObservation>{IMacObservation{GridCell{1, 1}, 1}};
  }

public:
  TestCoverageRobotThree(const GridCell &currentLoc, int timeBound, int xDim,
                         int yDim)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim} {}
};

class TestCoverageRobotFour : public CoverageRobot {

private:
  Action _planFn(const GridCell &currentLoc,
                 const std::vector<Action> &enabledActions, int ts,
                 int timeBound, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &covered,
                 const std::vector<IMacObservation> &currentObs) {
    return enabledActions.at(enabledActions.size() - 1);
  }

  ActionOutcome _executeFn(const GridCell &currentLoc, const Action &action) {
    return ActionOutcome{action, true,
                         GridCell{currentLoc.x, currentLoc.y + 1}};
  }

  std::vector<IMacObservation> _observeFn(const GridCell &currentLoc) {
    return std::vector<IMacObservation>{IMacObservation{GridCell{1, 1}, 1}};
  }

public:
  TestCoverageRobotFour(const GridCell &currentLoc, int timeBound, int xDim,
                        int yDim)
      : CoverageRobot{currentLoc, timeBound, xDim, yDim} {}
};

TEST_CASE("Tests for plan-execute-observe wrapper functions",
          "[plan-execute-observe]") {

  std::unique_ptr<TestCoverageRobotOne> robot{
      std::make_unique<TestCoverageRobotOne>(GridCell{2, 1}, 10, 5, 5)};

  REQUIRE(robot->planNextAction(1, nullptr, std::vector<IMacObservation>{}) ==
          Action::up);

  ActionOutcome outcome{robot->executeAction(Action::left)};
  REQUIRE(outcome.action == Action::left);
  REQUIRE(outcome.location.x == 2);
  REQUIRE(outcome.location.y == 1);
  REQUIRE(outcome.success == true);

  std::vector<IMacObservation> obsVector{robot->makeObservations()};
  REQUIRE(obsVector.size() == 1);
  REQUIRE(obsVector.at(0).cell.x == 2);
  REQUIRE(obsVector.at(0).cell.y == 1);
  REQUIRE(obsVector.at(0).occupied == 1);
}

TEST_CASE("Test for reset", "[resetForNextEpisode]") {

  std::unique_ptr<TestCoverageRobotOne> robot{
      std::make_unique<TestCoverageRobotOne>(GridCell{2, 1}, 0, 5, 5)};

  robot->runCoverageEpisode("/tmp/resetTestOne.csv");

  robot->resetForNextEpisode(GridCell{3, 3}, 10);

  robot->runCoverageEpisode("/tmp/resetTestTwo.csv");

  // Now check the two files
  // resetTestOne should be 2,1 once
  // resetTestTwo should be 3,3 ten times
  std::ifstream logOne{"/tmp/resetTestOne.csv"};
  std::string line{};
  std::vector<std::string> logVec{};
  if (logOne.is_open()) {
    while (getline(logOne, line)) {
      logVec.push_back(line);
    }
  }
  REQUIRE(logVec.size() == 1);
  REQUIRE(logVec.at(0) == "2,1");

  std::ifstream logTwo{"/tmp/resetTestTwo.csv"};
  logVec.clear();
  if (logTwo.is_open()) {
    while (getline(logTwo, line)) {
      logVec.push_back(line);
    }
  }
  REQUIRE(logVec.size() == 11);
  for (int i{0}; i < 11; ++i) {
    REQUIRE(logVec.at(i) == "3,3");
  }
}

TEST_CASE("Test for logCoveredLocations", "[logCoveredLocations]") {

  std::unique_ptr<TestCoverageRobotTwo> robot{
      std::make_unique<TestCoverageRobotTwo>(GridCell{2, 1}, 10, 5, 5)};

  robot->runCoverageEpisode("/tmp/logTest.csv");

  // Test log has got everything in the right order
  std::ifstream logOne{"/tmp/logTest.csv"};
  std::string line{};
  std::vector<std::string> logVec{};
  if (logOne.is_open()) {
    while (getline(logOne, line)) {
      logVec.push_back(line);
    }
  }
  REQUIRE(logVec.size() == 11);
  for (int i{0}; i < 11; ++i) {
    REQUIRE(logVec.at(i) == "2," + std::to_string(i + 1));
  }
}

TEST_CASE("Test for runCoverageEpisode", "[runCoverageEpisode]") {

  std::unique_ptr<TestCoverageRobotThree> robot{
      std::make_unique<TestCoverageRobotThree>(GridCell{2, 1}, 10, 5, 5)};

  robot->runCoverageEpisode("/tmp/runEpisodeTest.csv");
  // Test log has got everything in the right order
  std::ifstream logOne{"/tmp/runEpisodeTest.csv"};
  std::string line{};
  std::vector<std::string> logVec{};
  if (logOne.is_open()) {
    while (getline(logOne, line)) {
      logVec.push_back(line);
    }
  }
  REQUIRE(logVec.size() == 11);
  for (int i{0}; i < 11; ++i) {
    REQUIRE(logVec.at(i) == "2," + std::to_string(i + 1));
  }

  std::shared_ptr<IMac> imac{robot->getBIMac()->posteriorMean()};

  Eigen::MatrixXd init{imac->getInitialBelief()};
  Eigen::MatrixXd entry{imac->getEntryMatrix()};
  Eigen::MatrixXd exit{imac->getExitMatrix()};

  // Check the posterior has been computed correctly
  for (int i{0}; i < 5; ++i) {
    for (int j{0}; j < 5; ++j) {
      if (i == 1 and j == 1) {
        REQUIRE_THAT(init(i, j), Catch::Matchers::WithinRel(2.0 / 3.0, 0.001));
        REQUIRE_THAT(exit(i, j), Catch::Matchers::WithinRel(1.0 / 12.0, 0.001));
      } else {
        REQUIRE_THAT(init(i, j), Catch::Matchers::WithinRel(0.5, 0.001));
        REQUIRE_THAT(exit(i, j), Catch::Matchers::WithinRel(0.5, 0.001));
      }
      REQUIRE_THAT(entry(i, j), Catch::Matchers::WithinRel(0.5, 0.001));
    }
  }
}

TEST_CASE("Tests for _getEnabledActions function", "[getEnabledActions]") {

  std::unique_ptr<TestCoverageRobotFour> robot{
      std::make_unique<TestCoverageRobotFour>(GridCell{0, 0}, 10, 1, 1)};

  REQUIRE(robot->planNextAction(0, robot->getBIMac()->posteriorMean(),
                                std::vector<IMacObservation>{}) ==
          Action::wait);

  std::unique_ptr<TestCoverageRobotFour> robotTwo{
      std::make_unique<TestCoverageRobotFour>(GridCell{0, 0}, 10, 2, 1)};

  REQUIRE(robotTwo->planNextAction(0, robotTwo->getBIMac()->posteriorMean(),
                                   std::vector<IMacObservation>{}) ==
          Action::right);
}