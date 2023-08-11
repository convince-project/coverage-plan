/**
 * Unit tests for CoverageWorld.
 * @see coverage_world.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_world.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <memory>
#include <sstream>

TEST_CASE("Test for CoverageWorld::Connect", "[CoverageWorld::Connect]") {
  CoverageWorld world{GridCell{0, 1}, 2, 5, std::vector<GridCell>{}, nullptr};

  REQUIRE(world.Connect());
}

TEST_CASE("Test for CoverageWorld::Initialize", "[CoverageWorld::Initialize]") {
  Eigen::MatrixXd entryAndInit{Eigen::MatrixXd::Constant(2, 2, 1.0)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Constant(2, 2, 0.0)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndInit, exit, entryAndInit)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoverageWorld world{GridCell{0, 1}, 2, 5, std::vector<GridCell>{}, exec};

  despot::State *initState{world.Initialize()};
  CoverageState *coverState{static_cast<CoverageState *>(initState)};

  REQUIRE(coverState->robotPosition == GridCell{0, 1});
  REQUIRE(coverState->time == 2);
  REQUIRE(coverState->covered.size() == 1);
  REQUIRE(coverState->covered.count(GridCell{0, 1}) == 1);
  REQUIRE(coverState->map.size() == 4);
  REQUIRE(coverState->map(0, 0) == 1);
  REQUIRE(coverState->map(0, 1) == 1);
  REQUIRE(coverState->map(1, 0) == 0);
  REQUIRE(coverState->map(1, 1) == 1);
}

TEST_CASE("Test for CoverageWorld::GetCurrentState",
          "[CoverageWorld::GetCurrentState]") {
  Eigen::MatrixXd entryAndInit{Eigen::MatrixXd::Constant(2, 2, 1.0)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Constant(2, 2, 0.0)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndInit, exit, entryAndInit)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoverageWorld world{GridCell{0, 1}, 2, 5, std::vector<GridCell>{}, exec};

  despot::State *initState{world.Initialize()};

  despot::State *currentState{world.GetCurrentState()};

  REQUIRE(initState == currentState);
}

TEST_CASE("Test for CoverageWorld::PrintState", "[CoverageWorld::PrintState]") {

  Eigen::MatrixXd entryAndInit{Eigen::MatrixXd::Constant(2, 2, 1.0)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Constant(2, 2, 0.0)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndInit, exit, entryAndInit)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoverageWorld world{GridCell{0, 1}, 2, 5, std::vector<GridCell>{}, exec};

  despot::State *initState{world.Initialize()};

  std::ostringstream stream{};
  world.PrintState(*initState, stream);

  REQUIRE(stream.str() == initState->text());
}

TEST_CASE("Test for CoverageWorld::ExecuteAction with time bound exceeded",
          "[CoverageWorld::ExecuteAction-timeBound]") {
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd imacMat{3, 3};
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      imacMat(i, j) = i * 0.3 + j * 0.1;
    }
  }

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoverageWorld world{GridCell{0, 1}, 4, 5, fov, exec};

  world.Initialize();
  despot::OBS_TYPE obs{0};

  // Terminated as time bound reached
  REQUIRE(world.ExecuteAction(ActionHelpers::toInt(Action::right), obs));

  CoverageState *succ{static_cast<CoverageState *>(world.GetCurrentState())};
  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, fov, succ->robotPosition)};
  REQUIRE(succ->time == 5);

  if (succ->robotPosition == GridCell{0, 1}) { // Failure
    REQUIRE(succ->covered.count(GridCell{0, 1}) == 1);
    REQUIRE(succ->covered.size() == 1);
    REQUIRE(!std::get<1>(obsInfo));
  } else if (succ->robotPosition == GridCell{1, 1}) { // Sucess
    REQUIRE(succ->covered.count(GridCell{1, 1}) == 1);
    REQUIRE(succ->covered.size() == 2);
    REQUIRE(std::get<1>(obsInfo));
  } else {
    REQUIRE(false);
  }

  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    if (!imacObs.cell.outOfBounds(0, succ->map.cols(), 0, succ->map.rows())) {
      REQUIRE(succ->map(imacObs.cell.y, imacObs.cell.x) == imacObs.occupied);
    }
  }
}

TEST_CASE("Test for CoverageWorld::ExecuteAction with all covered",
          "[CoverageWorld::ExecuteAction-allCovered]") {
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd imacMat{1, 1};
  imacMat(0, 0) = 0.5;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoverageWorld world{GridCell{0, 0}, 1, 5, fov, exec};

  world.Initialize();
  despot::OBS_TYPE obs{0};

  // Terminated as time bound reached
  REQUIRE(world.ExecuteAction(ActionHelpers::toInt(Action::wait), obs));

  CoverageState *succ{static_cast<CoverageState *>(world.GetCurrentState())};
  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, fov, succ->robotPosition)};
  REQUIRE(succ->time == 2);
  REQUIRE(succ->covered.size() == 1);
  REQUIRE(succ->covered.count(GridCell{0, 0}) == 1);
  REQUIRE(std::get<1>(obsInfo));
  REQUIRE(succ->robotPosition == GridCell{0, 0});
  REQUIRE(succ->map(0, 0) == 0);

  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    REQUIRE(imacObs.occupied == 1);
  }
}

TEST_CASE("Test for CoverageWorld::ExecuteAction under normal use",
          "[CoverageWorld::ExecuteAction-normalUse]") {
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd imacMat{3, 3};
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      imacMat(i, j) = i * 0.3 + j * 0.1;
    }
  }

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoverageWorld world{GridCell{0, 1}, 1, 5, fov, exec};

  world.Initialize();
  despot::OBS_TYPE obs{0};

  // Terminated as time bound reached
  REQUIRE(!world.ExecuteAction(ActionHelpers::toInt(Action::down), obs));

  CoverageState *succ{static_cast<CoverageState *>(world.GetCurrentState())};
  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, fov, succ->robotPosition)};
  REQUIRE(succ->time == 2);

  if (succ->robotPosition == GridCell{0, 1}) { // Failure
    REQUIRE(succ->covered.count(GridCell{0, 1}) == 1);
    REQUIRE(succ->covered.size() == 1);
    REQUIRE(!std::get<1>(obsInfo));
  } else if (succ->robotPosition == GridCell{0, 2}) { // Success
    REQUIRE(succ->covered.count(GridCell{0, 2}) == 1);
    REQUIRE(succ->covered.size() == 2);
    REQUIRE(std::get<1>(obsInfo));
  } else {
    REQUIRE(false);
  }

  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    if (!imacObs.cell.outOfBounds(0, succ->map.cols(), 0, succ->map.rows())) {
      REQUIRE(succ->map(imacObs.cell.y, imacObs.cell.x) == imacObs.occupied);
    }
  }
}
