/**
 * Unit tests for BoustrophedonCoverageRobot.
 * @see boustrophedon_coverage_robot.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/baselines/boustrophedon_coverage_robot.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <fstream>
#include <memory>

TEST_CASE("Test for Boustrophedon when environment empty",
          "[BoustrophedonCoverageRobot::allEmpty]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 3)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 3)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 3)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  std::unique_ptr<BoustrophedonCoverageRobot> robot{
      std::make_unique<BoustrophedonCoverageRobot>(GridCell{0, 0}, 10, 3, 3,
                                                   fov, exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/allEmpty.csv")};
  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(1.0, 0.001));

  // Check the path
  std::ifstream visitedFile{"/tmp/allEmpty.csv"};
  std::vector<std::string> expected{"0,0", "0,1", "0,2", "1,2", "1,1",
                                    "1,0", "2,0", "2,1", "2,2"};
  std::string currentLine{};
  int lineNum{0};
  while (getline(visitedFile, currentLine)) {
    REQUIRE(currentLine == expected.at(lineNum));
    ++lineNum;
  }
  visitedFile.close();
  REQUIRE(lineNum == 9);
}

TEST_CASE("Test for Boustrophedon when obstacle present",
          "[BoustrophedonCoverageRobot::withObstacle]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 3)};
  entry(0, 1) = 1;
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 3)};
  exit(0, 1) = 0;
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 3)};
  init(0, 1) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  std::unique_ptr<BoustrophedonCoverageRobot> robot{
      std::make_unique<BoustrophedonCoverageRobot>(GridCell{0, 0}, 10, 3, 3,
                                                   fov, exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/withObstacle.csv")};
  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(7.0 / 9.0, 0.001));

  // Check the path
  std::ifstream visitedFile{"/tmp/withObstacle.csv"};
  std::vector<std::string> expected{"0,0", "0,1", "0,2", "1,2", "1,1", "2,1",
                                    "2,0", "2,0", "2,0", "2,0", "2,0"};
  std::string currentLine{};
  int lineNum{0};
  while (getline(visitedFile, currentLine)) {
    REQUIRE(currentLine == expected.at(lineNum));
    ++lineNum;
  }
  visitedFile.close();
  REQUIRE(lineNum == 11);
}

TEST_CASE("Test for Boustrophedon with dynamic obstacle",
          "[BoustrophedonCoverageRobot::dynamicObstacle]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(2, 1)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(2, 1)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(2, 1)};
  init(1, 0) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  std::unique_ptr<BoustrophedonCoverageRobot> robot{
      std::make_unique<BoustrophedonCoverageRobot>(GridCell{0, 0}, 2, 1, 2, fov,
                                                   exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/dynamicObstacle.csv")};
  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(1.0, 0.001));

  // Check the path
  std::ifstream visitedFile{"/tmp/dynamicObstacle.csv"};
  std::vector<std::string> expected{"0,0", "0,0", "0,1"};
  std::string currentLine{};
  int lineNum{0};
  while (getline(visitedFile, currentLine)) {
    REQUIRE(currentLine == expected.at(lineNum));
    ++lineNum;
  }
  visitedFile.close();
  REQUIRE(lineNum == 3);
}