/**
 * Unit tests for POMDPCoverageRobot.
 * @see pomdp_coverage_robot.h/.cpp
 *
 * I'm a little limited in how much I can unit test here but I'll try as much
 * as I can.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/pomdp_coverage_robot.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <iostream>
#include <memory>
#include <vector>

TEST_CASE("Tests for POMDPCoverageRobot setup cleanup, and makeObservations",
          "[POMDPCoverageRobot::episodeSetup/Cleanup]") {
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(5, 5)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(5, 5)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(5, 5)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  POMDPCoverageRobot robot{GridCell{1, 0}, 5, 5, 5, fov, exec};

  // A lot of allocating goes on here
  robot.episodeSetup(GridCell{1, 0}, 0, 5, imac);

  std::vector<IMacObservation> obs{robot.makeObservations()};
  REQUIRE(obs.size() == 4);
  REQUIRE(obs.at(0).cell == GridCell{0, 0});
  REQUIRE(obs.at(0).occupied == 0);
  REQUIRE(obs.at(1).cell == GridCell{2, 0});
  REQUIRE(obs.at(1).occupied == 0);
  REQUIRE(obs.at(2).cell == GridCell{1, -1});
  REQUIRE(obs.at(2).occupied == 1);
  REQUIRE(obs.at(3).cell == GridCell{1, 1});
  REQUIRE(obs.at(3).occupied == 0);

  // Basically just need to ensure nothing breaks here
  // And that valgrind is vaguely happy
  robot.episodeCleanup();

  // Observation vector should have been reset after cleanup
  REQUIRE(robot.makeObservations().size() == 0);
}

TEST_CASE("Tests for POMDPCoverageRobot executeAction with failure",
          "[POMDPCoverageRobot-executeAction-fail]") {

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

  std::unique_ptr<POMDPCoverageRobot> robot{
      std::make_unique<POMDPCoverageRobot>(GridCell{0, 0}, 2, 2, 1,
                                           std::vector<GridCell>{}, exec)};

  robot->episodeSetup(GridCell{0, 0}, 0, 2, imac);
  ActionOutcome outcome{robot->executeAction(Action::right)};

  // Always empty
  REQUIRE(outcome.action == Action::right);
  REQUIRE(outcome.success == false);
  REQUIRE(outcome.location.x == 0);
  REQUIRE(outcome.location.y == 0);
}

TEST_CASE(
    "Tests for POMDPCoverageRobot executeAction with out of bounds failure",
    "[POMDPCoverageRobot-executeAction-outOfBounds]") {

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

  std::unique_ptr<POMDPCoverageRobot> robot{
      std::make_unique<POMDPCoverageRobot>(GridCell{0, 0}, 2, 2, 1,
                                           std::vector<GridCell>{}, exec)};

  robot->episodeSetup(GridCell{0, 0}, 0, 2, imac);
  ActionOutcome outcome{robot->executeAction(Action::left)};

  // Always empty
  REQUIRE(outcome.action == Action::left);
  REQUIRE(outcome.success == false);
  REQUIRE(outcome.location.x == 0);
  REQUIRE(outcome.location.y == 0);
}

TEST_CASE("Tests for POMDPCoverageRobot executeAction with success",
          "[POMDPCoverageRobot-executeAction-succeed]") {

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

  std::unique_ptr<POMDPCoverageRobot> robot{
      std::make_unique<POMDPCoverageRobot>(GridCell{0, 0}, 2, 2, 1,
                                           std::vector<GridCell>{}, exec)};

  robot->episodeSetup(GridCell{0, 0}, 0, 2, imac);
  ActionOutcome outcome{robot->executeAction(Action::right)};

  // Always empty
  REQUIRE(outcome.action == Action::right);
  REQUIRE(outcome.success == true);
  REQUIRE(outcome.location.x == 1);
  REQUIRE(outcome.location.y == 0);
}

TEST_CASE("Tests for POMDPCoverageRobot planning",
          "[POMDPCoverageRobot::planNextAction]") {
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(5, 5)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(5, 5)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(5, 5)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  POMDPCoverageRobot robot{GridCell{1, 0}, 5, 5, 5, fov, exec};
  robot.episodeSetup(GridCell{1, 0}, 0, 5, imac);

  // Just checking a valid action is output
  REQUIRE_NOTHROW(robot.planNextAction(0, imac, robot.makeObservations()));
}

TEST_CASE("Tests for POMDPCoverageRobot bound setting",
          "[POMDPCoverageRobot::boundSetting]") {
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(5, 5)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(5, 5)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(5, 5)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  POMDPCoverageRobot robot{GridCell{1, 0}, 5, 5, 5, fov, exec, imac, "TRIVIAL"};
  std::cout << "Bound Setting Test: Bounds Should be Trivial:\n";
  robot.episodeSetup(GridCell{1, 0}, 0, 5, imac);
}