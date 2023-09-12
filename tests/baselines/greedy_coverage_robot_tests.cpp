/**
 * Unit tests for GreedyCoverageRobot.
 * @see greedy_coverage_robot.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/baselines/greedy_coverage_robot.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <memory>

TEST_CASE("Test for GreedyCoverageRobot when no cells visited",
          "[GreedyCoverageRobot::noVisited]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Constant(3, 3, 0.5)};
  entry(0, 1) = 0.9;
  entry(1, 0) = 0.5;
  entry(1, 2) = 0.2;
  entry(2, 1) = 0.85;

  Eigen::MatrixXd exit{Eigen::MatrixXd::Constant(3, 3, 0.5)};
  exit(0, 1) = 0.6;
  exit(1, 0) = 0.1;
  exit(1, 2) = 0.15;
  exit(2, 1) = 0.4;

  Eigen::MatrixXd init{Eigen::MatrixXd::Constant(3, 3, 0.5)};
  init(0, 1) = 1.0;
  init(1, 0) = 0.0;
  init(1, 2) = 0.0;
  init(2, 1) = 1.0;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::unique_ptr<GreedyCoverageRobot> robot{
      std::make_unique<GreedyCoverageRobot>(GridCell{1, 1}, 5, 3, 3,
                                            std::vector<GridCell>{}, exec)};

  robot->episodeSetup(GridCell{1, 1}, 0, 5, imac);
  Action act{robot->planNextAction(0, imac, std::vector<IMacObservation>{})};
  robot->episodeCleanup();

  REQUIRE(act == Action::right);
}

TEST_CASE(
    "Test for GreedyCoverageRobot which tests visited nodes aren't chosen",
    "[GreedyCoverageRobot::visited]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 1)};

  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 1)};

  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 1)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::unique_ptr<GreedyCoverageRobot> robot{
      std::make_unique<GreedyCoverageRobot>(GridCell{0, 0}, 2, 1, 3,
                                            std::vector<GridCell>{}, exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/dummy.csv")};

  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(1.0, 0.001));
}

TEST_CASE(
    "Test for GreedyCoverageRobot when we keep going after visiting everything",
    "[GreedyCoverageRobot::finishEarly]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 1)};

  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 1)};

  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 1)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::unique_ptr<GreedyCoverageRobot> robot{
      std::make_unique<GreedyCoverageRobot>(GridCell{0, 0}, 5, 1, 3,
                                            std::vector<GridCell>{}, exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/dummy.csv")};

  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(1.0, 0.001));
}