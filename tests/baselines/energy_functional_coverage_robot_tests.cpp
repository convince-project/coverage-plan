/**
 * Unit tests for EnergyFunctionalCoverageRobot.
 * @see energy_functional_coverage_robot.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/baselines/energy_functional_coverage_robot.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <fstream>
#include <memory>

TEST_CASE("Test for Energy Functional when robot should go down/up",
          "[EnergyFunctionalCoverageRobot::downUp]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 3)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 3)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 3)};
  init(0, 1) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  std::unique_ptr<EnergyFunctionalCoverageRobot> robot{
      std::make_unique<EnergyFunctionalCoverageRobot>(GridCell{0, 0}, 10, 3, 3,
                                                      fov, exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/downUp.csv")};
  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(1.0, 0.001));

  // Check the path
  std::ifstream visitedFile{"/tmp/downUp.csv"};
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

TEST_CASE("Test for Energy Functional when robot should go left/right",
          "[EnergyFunctionalCoverageRobot::leftRight]") {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 3)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 3)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 3)};
  init(1, 0) = 1;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, init)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  std::unique_ptr<EnergyFunctionalCoverageRobot> robot{
      std::make_unique<EnergyFunctionalCoverageRobot>(GridCell{0, 0}, 10, 3, 3,
                                                      fov, exec)};

  double propCovered{robot->runCoverageEpisode("/tmp/leftRight.csv")};
  REQUIRE_THAT(propCovered, Catch::Matchers::WithinRel(1.0, 0.001));

  // Check the path
  std::ifstream visitedFile{"/tmp/leftRight.csv"};
  std::vector<std::string> expected{"0,0", "1,0", "2,0", "2,1", "1,1",
                                    "0,1", "0,2", "1,2", "2,2"};
  std::string currentLine{};
  int lineNum{0};
  while (getline(visitedFile, currentLine)) {
    REQUIRE(currentLine == expected.at(lineNum));
    ++lineNum;
  }
  visitedFile.close();
  REQUIRE(lineNum == 9);
}