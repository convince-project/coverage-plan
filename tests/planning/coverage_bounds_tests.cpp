/**
 * Unit tests for the coverage bounds in coverage_bound.h/.cpp.
 * @see coverage_bound.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_bounds.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/planning/coverage_state.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <despot/core/globals.h>
#include <despot/core/history.h>
#include <despot/interface/default_policy.h>
#include <despot/interface/pomdp.h>
#include <despot/random_streams.h>
#include <memory>
#include <set>
#include <vector>

TEST_CASE("Tests for MaxCellsUpperBound", "[MaxCellsUpperBound]") {
  MaxCellsUpperBound bound{10, 15};

  CoverageState state{GridCell{0, 0}, 0, Eigen::MatrixXi::Zero(3, 3),
                      std::set<GridCell>{}, 1.0};

  REQUIRE_THAT(bound.Value(state), Catch::Matchers::WithinRel(10.0, 0.001));

  state.time = 12;
  REQUIRE_THAT(bound.Value(state), Catch::Matchers::WithinRel(3.0, 0.001));

  state.time = 15;
  REQUIRE_THAT(bound.Value(state), Catch::Matchers::WithinRel(0.0, 0.001));

  state.time = 8;
  for (int i{0}; i < 8; ++i) {
    state.covered.insert(GridCell{i, i});
  }
  REQUIRE_THAT(bound.Value(state), Catch::Matchers::WithinRel(2.0, 0.001));
}

TEST_CASE("Tests for ZeroParticleLowerBound", "[ZeroParticleLowerBound]") {
  ZeroParticleLowerBound bound{};

  despot::ValuedAction va{bound.Value(std::vector<despot::State *>{})};

  REQUIRE(ActionHelpers::fromInt(va.action) == Action::up);
  REQUIRE(va.value == 0.0);
}

TEST_CASE("Tests for GreedyCoverageDefaultPolicy",
          "[GreedyCoverageDefaultPolicy]") {

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  Eigen::MatrixXd imacMat{3, 3};
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      imacMat(i, j) = i * 0.3 + j * 0.1;
    }
  }

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};

  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(fov, imac, 5)};

  ZeroParticleLowerBound zeroBound{};

  GreedyCoverageDefaultPolicy policy{pomdp.get(), &zeroBound, imac};

  despot::History history{};
  despot::RandomStreams streams{3, 10};

  // Test 1: All states covered, should get random action
  std::vector<despot::State *> particles{};
  for (int i{0}; i < 5; ++i) {
    CoverageState *state{
        static_cast<CoverageState *>(pomdp->Allocate(-1, 0.2))};
    state->robotPosition = GridCell{1, 1};
    state->time = 3;
    state->map = Eigen::MatrixXi::Zero(3, 3);
    for (int x{0}; x < 3; ++x) {
      for (int y{0}; y < 3; ++y) {
        state->covered.insert(GridCell{x, y});
      }
    }
    particles.push_back(state);
  }

  despot::ACT_TYPE action{policy.Action(particles, streams, history)};
  REQUIRE(action >= 0);
  REQUIRE(action <= 4);

  // Checking we actually get some randomness here
  std::set<despot::ACT_TYPE> actSet{};
  for (int i{0}; i < 100; ++i) {
    actSet.insert(policy.Action(particles, streams, history));
  }
  REQUIRE(actSet.size() > 1);
  REQUIRE(actSet.size() <= 5);

  for (despot::State *state : particles) {
    pomdp->Free(state);
  }

  // Test 2: All states the same but single good action.
  particles.clear();
  for (int i{0}; i < 5; ++i) {
    CoverageState *state{
        static_cast<CoverageState *>(pomdp->Allocate(-1, 0.2))};
    state->robotPosition = GridCell{1, 1};
    state->time = 3;
    state->map = Eigen::MatrixXi::Zero(3, 3);
    state->map(1, 0) = 1;
    state->map(2, 1) = 1;
    state->covered = std::set<GridCell>{GridCell{1, 1}};
    particles.push_back(state);
  }

  action = policy.Action(particles, streams, history);
  REQUIRE(ActionHelpers::fromInt(action) == Action::up);

  // Test 3: Make best action lead to a covered cell, giving us different action
  for (int i{0}; i < 5; ++i) {
    CoverageState *state{static_cast<CoverageState *>(particles.at(i))};
    state->covered.insert(GridCell{1, 0});
  }

  action = policy.Action(particles, streams, history);
  REQUIRE(ActionHelpers::fromInt(action) == Action::down);
  for (despot::State *state : particles) {
    pomdp->Free(state);
  }

  // Test 4: Two states with different weights
  particles.clear();

  CoverageState *stateOne{
      static_cast<CoverageState *>(pomdp->Allocate(-1, 0.4))};
  stateOne->robotPosition = GridCell{1, 1};
  stateOne->time = 3;
  stateOne->map = Eigen::MatrixXi::Zero(3, 3);
  stateOne->map(1, 0) = 1;
  stateOne->map(2, 1) = 1;
  stateOne->covered = std::set<GridCell>{GridCell{1, 1}};
  particles.push_back(stateOne);

  CoverageState *stateTwo{
      static_cast<CoverageState *>(pomdp->Allocate(-1, 0.6))};
  stateTwo->robotPosition = GridCell{2, 1};
  stateTwo->time = 3;
  stateTwo->map = Eigen::MatrixXi::Zero(3, 3);
  stateTwo->map(2, 2) = 1;
  stateTwo->map(0, 2) = 1;
  stateTwo->covered = std::set<GridCell>{GridCell{2, 1}};
  particles.push_back(stateTwo);

  action = policy.Action(particles, streams, history);
  REQUIRE(ActionHelpers::fromInt(action) == Action::down);
  for (despot::State *state : particles) {
    pomdp->Free(state);
  }
}