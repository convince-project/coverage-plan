/**
 * Tests for the RandomCoverageRobot class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_world.h"
#include "coverage_plan/planning/random_coverage_robot.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <memory>
#include <vector>

TEST_CASE("Tests for RandomCoverageRobot planNextAction",
          "[RandomCoverageRobot-planNextAction]") {

  Eigen::MatrixXd entry{1, 2};
  entry(0, 0) = 0;
  entry(0, 1) = 1;

  Eigen::MatrixXd exit{1, 2};
  exit(0, 0) = 1;
  exit(0, 1) = 0;

  Eigen::MatrixXd init{1, 2};
  init(0, 0) = 0;
  init(0, 1) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::shared_ptr<CoverageWorld> world{std::make_shared<CoverageWorld>(
      GridCell{0, 0}, 0, 2, std::vector<GridCell>{}, exec)};

  std::unique_ptr<RandomCoverageRobot> robot{
      std::make_unique<RandomCoverageRobot>(GridCell{0, 0}, 2, 1, 1, world,
                                            std::vector<GridCell>{})};

  Action action{
      robot->planNextAction(1, nullptr, std::vector<IMacObservation>{})};

  // Can't do much more than this
  REQUIRE(action == Action::wait);
}

TEST_CASE("Tests for RandomCoverageRobot executeAction with failure",
          "[RandomCoverageRobot-executeAction-fail]") {

  Eigen::MatrixXd entry{1, 2};
  entry(0, 0) = 0;
  entry(0, 1) = 1;

  Eigen::MatrixXd exit{1, 2};
  exit(0, 0) = 1;
  exit(0, 1) = 0;

  Eigen::MatrixXd init{1, 2};
  init(0, 0) = 0;
  init(0, 1) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::shared_ptr<CoverageWorld> world{std::make_shared<CoverageWorld>(
      GridCell{0, 0}, 0, 2, std::vector<GridCell>{}, exec)};

  std::unique_ptr<RandomCoverageRobot> robot{
      std::make_unique<RandomCoverageRobot>(GridCell{0, 0}, 2, 2, 1, world,
                                            std::vector<GridCell>{})};

  robot->episodeSetup(GridCell{0, 0}, 0, 2, nullptr);
  ActionOutcome outcome{robot->executeAction(Action::right)};

  // Always empty
  REQUIRE(outcome.action == Action::right);
  REQUIRE(outcome.success == false);
  REQUIRE(outcome.location.x == 0);
  REQUIRE(outcome.location.y == 0);
}

TEST_CASE(
    "Tests for RandomCoverageRobot executeAction with out of bounds failure",
    "[RandomCoverageRobot-executeAction-outOfBounds]") {

  Eigen::MatrixXd entry{1, 2};
  entry(0, 0) = 0;
  entry(0, 1) = 1;

  Eigen::MatrixXd exit{1, 2};
  exit(0, 0) = 1;
  exit(0, 1) = 0;

  Eigen::MatrixXd init{1, 2};
  init(0, 0) = 0;
  init(0, 1) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::shared_ptr<CoverageWorld> world{std::make_shared<CoverageWorld>(
      GridCell{0, 0}, 0, 2, std::vector<GridCell>{}, exec)};

  std::unique_ptr<RandomCoverageRobot> robot{
      std::make_unique<RandomCoverageRobot>(GridCell{0, 0}, 2, 2, 1, world,
                                            std::vector<GridCell>{})};

  robot->episodeSetup(GridCell{0, 0}, 0, 2, nullptr);
  ActionOutcome outcome{robot->executeAction(Action::left)};

  // Always empty
  REQUIRE(outcome.action == Action::left);
  REQUIRE(outcome.success == false);
  REQUIRE(outcome.location.x == 0);
  REQUIRE(outcome.location.y == 0);
}

TEST_CASE("Tests for RandomCoverageRobot executeAction with success",
          "[RandomCoverageRobot-executeAction-succeed]") {

  Eigen::MatrixXd entry{1, 2};
  entry(0, 0) = 0;
  entry(0, 1) = 0;

  Eigen::MatrixXd exit{1, 2};
  exit(0, 0) = 1;
  exit(0, 1) = 1;

  Eigen::MatrixXd init{1, 2};
  init(0, 0) = 0;
  init(0, 1) = 0;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::shared_ptr<CoverageWorld> world{std::make_shared<CoverageWorld>(
      GridCell{0, 0}, 0, 2, std::vector<GridCell>{}, exec)};

  std::unique_ptr<RandomCoverageRobot> robot{
      std::make_unique<RandomCoverageRobot>(GridCell{0, 0}, 2, 2, 1, world,
                                            std::vector<GridCell>{})};

  robot->episodeSetup(GridCell{0, 0}, 0, 2, nullptr);
  ActionOutcome outcome{robot->executeAction(Action::right)};

  // Always empty
  REQUIRE(outcome.action == Action::right);
  REQUIRE(outcome.success == true);
  REQUIRE(outcome.location.x == 1);
  REQUIRE(outcome.location.y == 0);
}

TEST_CASE("Tests for RandomCoverageRobot makeObservations",
          "[RandomCoverageRobot-makeObservations]") {

  Eigen::MatrixXd entry{1, 2};
  entry(0, 0) = 0;
  entry(0, 1) = 1;

  Eigen::MatrixXd exit{1, 2};
  exit(0, 0) = 1;
  exit(0, 1) = 0;

  Eigen::MatrixXd init{1, 2};
  init(0, 0) = 0;
  init(0, 1) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::shared_ptr<CoverageWorld> world{std::make_shared<CoverageWorld>(
      GridCell{0, 0}, 0, 2, std::vector<GridCell>{}, exec)};

  std::unique_ptr<RandomCoverageRobot> robot{
      std::make_unique<RandomCoverageRobot>(GridCell{0, 0}, 2, 1, 1, world,
                                            std::vector<GridCell>{})};

  std::vector<IMacObservation> obsVector{robot->makeObservations()};

  // Always empty
  REQUIRE(obsVector.size() == 0);
}
