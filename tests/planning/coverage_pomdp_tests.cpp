/**
 * Unit tests for CoveragePOMDP in coverage_pomdp.h.
 * @see coverage_pomdp.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include <catch2/catch.hpp>
#include <despot/interface/default_policy.h>
#include <memory>
#include <vector>

// TODO: Step

TEST_CASE("Test for CoveragePOMDP::NumActions", "[CoveragePOMDP::NumActions]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  REQUIRE(pomdp->NumActions() == 5);
}

// TODO: ObsProb

// TODO: Initial Belief

TEST_CASE("Test for CoveragePOMDP::GetMaxReward",
          "[CoveragePOMDP::GetMaxReward]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  REQUIRE(pomdp->GetMaxReward() == 1.0);
}

TEST_CASE("Test for CoveragePOMDP::GetBestAction",
          "[CoveragePOMDP::GetBestAction]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  despot::ValuedAction bestAction{pomdp->GetBestAction()};
  REQUIRE(bestAction.action == ActionHelpers::toInt(Action::up));
  REQUIRE(bestAction.value == 0.0);
}

// TODO: Printers

// TODO: Allocate/Free

// TODO: Copy

TEST_CASE("Test for CoveragePOMDP::NumActiveParticles",
          "[CoveragePOMDP::NumActiveParticles]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  REQUIRE(pomdp->NumActiveParticles() == 0);
  despot::State *stateOne{pomdp->Allocate()};
  REQUIRE(pomdp->NumActiveParticles() == 1);
  despot::State *stateTwo{pomdp->Allocate()};
  REQUIRE(pomdp->NumActiveParticles() == 2);
  pomdp->Free(stateTwo);
  REQUIRE(pomdp->NumActiveParticles() == 1);
  pomdp->Free(stateOne);
  REQUIRE(pomdp->NumActiveParticles() == 0);
}
