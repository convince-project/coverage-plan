/**
 * Tests for the CoverageRobot class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/coverage_robot.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <filesystem>
#include <fstream>
#include <vector>

TEST_CASE("Tests for plan-execute-observe wrapper functions",
          "[plan-execute-observe]") {

  auto planLambda{
      [](const GridCell &cell, const std::vector<Action> &validActions,
         int time, int timeBound, std::shared_ptr<IMac> imac,
         const std::vector<GridCell> &covered,
         const std::vector<IMacObservation> &obs) { return Action::up; }};
  auto actLambda{[](const GridCell &cell, const Action &action) {
    return ActionOutcome{action, true, cell};
  }};
  auto observeLambda{[](const GridCell &cell) {
    return std::vector<IMacObservation>{IMacObservation{cell, 1}};
  }};

  std::unique_ptr<CoverageRobot> robot{std::make_unique<CoverageRobot>(
      GridCell{2, 1}, 10, 5, 5, planLambda, actLambda, observeLambda)};

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

  auto planLambda{
      [](const GridCell &cell, const std::vector<Action> &validActions,
         int time, int timeBound, std::shared_ptr<IMac> imac,
         const std::vector<GridCell> &covered,
         const std::vector<IMacObservation> &obs) { return Action::up; }};
  auto actLambda{[](const GridCell &cell, const Action &action) {
    return ActionOutcome{action, true, cell};
  }};
  auto observeLambda{[](const GridCell &cell) {
    return std::vector<IMacObservation>{IMacObservation{cell, 1}};
  }};

  std::unique_ptr<CoverageRobot> robot{std::make_unique<CoverageRobot>(
      GridCell{2, 1}, 0, 5, 5, planLambda, actLambda, observeLambda)};

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

  auto planLambda{
      [](const GridCell &cell, const std::vector<Action> &validActions,
         int time, int timeBound, std::shared_ptr<IMac> imac,
         const std::vector<GridCell> &covered,
         const std::vector<IMacObservation> &obs) { return Action::up; }};
  auto actLambda{[](const GridCell &cell, const Action &action) {
    return ActionOutcome{action, true, GridCell{cell.x, cell.y + 1}};
  }};
  auto observeLambda{[](const GridCell &cell) {
    return std::vector<IMacObservation>{IMacObservation{cell, 1}};
  }};

  std::unique_ptr<CoverageRobot> robot{std::make_unique<CoverageRobot>(
      GridCell{2, 1}, 10, 5, 5, planLambda, actLambda, observeLambda)};

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
  auto planLambda{
      [](const GridCell &cell, const std::vector<Action> &validActions,
         int time, int timeBound, std::shared_ptr<IMac> imac,
         const std::vector<GridCell> &covered,
         const std::vector<IMacObservation> &obs) { return Action::up; }};
  auto actLambda{[](const GridCell &cell, const Action &action) {
    return ActionOutcome{action, true, GridCell{cell.x, cell.y + 1}};
  }};
  auto observeLambda{[](const GridCell &cell) {
    return std::vector<IMacObservation>{IMacObservation{GridCell{1, 1}, 1}};
  }};

  std::unique_ptr<CoverageRobot> robot{std::make_unique<CoverageRobot>(
      GridCell{2, 1}, 10, 5, 5, planLambda, actLambda, observeLambda)};

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
  auto planLambda{[](const GridCell &cell,
                     const std::vector<Action> &validActions, int time,
                     int timeBound, std::shared_ptr<IMac> imac,
                     const std::vector<GridCell> &covered,
                     const std::vector<IMacObservation> &obs) {
    return validActions.at(validActions.size() - 1);
  }};
  auto actLambda{[](const GridCell &cell, const Action &action) {
    return ActionOutcome{action, true, GridCell{cell.x, cell.y + 1}};
  }};
  auto observeLambda{[](const GridCell &cell) {
    return std::vector<IMacObservation>{IMacObservation{GridCell{1, 1}, 1}};
  }};

  std::unique_ptr<CoverageRobot> robot{std::make_unique<CoverageRobot>(
      GridCell{0, 0}, 10, 1, 1, planLambda, actLambda, observeLambda)};

  REQUIRE(robot->planNextAction(0, robot->getBIMac()->posteriorMean(),
                                std::vector<IMacObservation>{}) ==
          Action::wait);

  std::unique_ptr<CoverageRobot> robotTwo{std::make_unique<CoverageRobot>(
      GridCell{0, 0}, 10, 2, 1, planLambda, actLambda, observeLambda)};

  REQUIRE(robotTwo->planNextAction(0, robotTwo->getBIMac()->posteriorMean(),
                                   std::vector<IMacObservation>{}) ==
          Action::right);
}