/**
 * Unit tests for CoveragePlanner.
 * @see coverage_planner.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/coverage_planner.h"
#include "coverage_plan/planning/coverage_state.h"
#include <catch2/catch.hpp>
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

TEST_CASE("Test for CoveragePlanner::InitializeModel",
          "[CoveragePlanner::InitializeModel]") {
  GridCell initPos{1, 1};
  int initTime{0};
  int timeBound{5};
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{0, 1}};

  Eigen::MatrixXd imacMat{Eigen::MatrixXd::Zero(3, 3)};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoveragePlanner planner{initPos, initTime, timeBound, fov, exec, imac};

  despot::DSPOMDP *pomdp{planner.InitializeModel(nullptr)};

  // A couple of small checks in the object to check we've got the right class
  REQUIRE(pomdp->NumActions() == 5);
  REQUIRE(pomdp->GetMaxReward() == 1.0);

  // Deallocate everything
  delete pomdp;
}

TEST_CASE("Test for CoveragePlanner::InitializeWorld",
          "[CoveragePlanner::InitializeWorld]") {
  GridCell initPos{1, 1};
  int initTime{0};
  int timeBound{5};
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{0, 1}};

  Eigen::MatrixXd imacMat{Eigen::MatrixXd::Zero(3, 3)};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoveragePlanner planner{initPos, initTime, timeBound, fov, exec, imac};

  std::string type{"DEFAULT"};
  despot::World *world{planner.InitializeWorld(type, nullptr, nullptr)};

  // A couple of small checks in the object to check we've got the right class
  REQUIRE(world->Connect());
  despot::State *initState{world->Initialize()};
  CoverageState *coverState{static_cast<CoverageState *>(initState)};

  REQUIRE(coverState->robotPosition == GridCell{1, 1});
  REQUIRE(coverState->time == 0);
  REQUIRE(coverState->covered == std::set<GridCell>{GridCell{1, 1}});
  REQUIRE(coverState->map.size() == 9);

  // Deallocate everything
  delete world;
}

TEST_CASE("Test for CoveragePlanner::InitializeDefaultParameters",
          "[CoveragePlanner::InitializeDefaultParameters]") {
  GridCell initPos{1, 1};
  int initTime{0};
  int timeBound{5};
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{0, 1}};

  Eigen::MatrixXd imacMat{Eigen::MatrixXd::Zero(3, 3)};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoveragePlanner planner{initPos, initTime, timeBound, fov, exec, imac};

  despot::Globals::config = despot::Config();

  planner.InitializeDefaultParameters();

  REQUIRE(despot::Globals::config.time_per_move == 1);
  REQUIRE(despot::Globals::config.sim_len == 6);
  REQUIRE(despot::Globals::config.num_scenarios == 500);
  REQUIRE(despot::Globals::config.search_depth == 6);
  REQUIRE(despot::Globals::config.max_policy_sim_len == 6);
  REQUIRE(despot::Globals::config.discount == 0.99999);
  REQUIRE(despot::Globals::config.pruning_constant == 0.01);
  REQUIRE(despot::Globals::config.xi == 0.95);
  REQUIRE(despot::Globals::config.root_seed != 42);
  REQUIRE(despot::Globals::config.default_action == "");
  REQUIRE(despot::Globals::config.noise == 0.1);
  REQUIRE(!despot::Globals::config.silence);
}

TEST_CASE("Test for CoveragePlanner::ChooseSolver",
          "[CoveragePlanner::ChooseSolver]") {
  GridCell initPos{1, 1};
  int initTime{0};
  int timeBound{5};
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{0, 1}};

  Eigen::MatrixXd imacMat{Eigen::MatrixXd::Zero(3, 3)};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  CoveragePlanner planner{initPos, initTime, timeBound, fov, exec, imac};

  REQUIRE(planner.ChooseSolver() == "DESPOT");
}