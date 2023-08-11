/**
 * Unit tests for CoverageBelief in coverage_belief.h
 * @see coverage_belief.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/planning/coverage_state.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <despot/interface/belief.h>
#include <despot/interface/pomdp.h>
#include <memory>
#include <vector>

TEST_CASE("Tests for CoverageBelief::Sample.", "[CoverageBelief::Sample]") {

  Eigen::MatrixXd entry{1, 2};
  Eigen::MatrixXd exit{1, 2};
  Eigen::MatrixXd initBelief{1, 2};
  entry(0, 0) = 1;
  entry(0, 1) = 0.5;
  exit(0, 0) = 0;
  exit(0, 1) = 0.5;
  initBelief(0, 0) = 0;
  initBelief(0, 1) = 1;
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, initBelief)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  const CoveragePOMDP *pomdp{new CoveragePOMDP{fov, imac, 5}};

  std::unique_ptr<CoverageBelief> belief{std::make_unique<CoverageBelief>(
      pomdp, GridCell{0, 0}, 1, std::set<GridCell>{GridCell{0, 0}},
      imac->getInitialBelief(), imac, fov)};

  // Sample 5 states
  std::vector<despot::State *> particles{belief->Sample(5)};

  REQUIRE(particles.size() == 5);

  for (despot::State *state : particles) {
    CoverageState *coverState{static_cast<CoverageState *>(state)};
    REQUIRE(coverState->robotPosition == GridCell{0, 0});
    REQUIRE(coverState->time == 1);
    REQUIRE(coverState->covered.size() == 1);
    REQUIRE(coverState->covered.count(GridCell{0, 0}) == 1);
    REQUIRE(coverState->map.size() == 2);
    REQUIRE(coverState->map(0, 0) == 0);
    REQUIRE((coverState->map(0, 1) == 0 || coverState->map(0, 1) == 1));
    REQUIRE(coverState->state_id == -1);
    REQUIRE(coverState->weight == (1.0 / 5.0));
  }

  // deallocate everything
  for (despot::State *state : particles) {
    pomdp->Free(state);
  }
  delete pomdp;
}

TEST_CASE("Tests for CoverageBelief::MakeCopy", "[CoverageBelief::MakeCopy]") {

  Eigen::MatrixXd entry{1, 2};
  Eigen::MatrixXd exit{1, 2};
  Eigen::MatrixXd initBelief{1, 2};
  entry(0, 0) = 1;
  entry(0, 1) = 1;
  exit(0, 0) = 0;
  exit(0, 1) = 0;
  initBelief(0, 0) = 0;
  initBelief(0, 1) = 1;
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, initBelief)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  const CoveragePOMDP *pomdp{new CoveragePOMDP{fov, imac, 5}};

  // Test copying using sampling and deterministic belief
  std::unique_ptr<CoverageBelief> belief{std::make_unique<CoverageBelief>(
      pomdp, GridCell{0, 0}, 1, std::set<GridCell>{GridCell{0, 0}},
      imac->getInitialBelief(), imac, fov)};

  std::vector<despot::State *> particles{belief->Sample(1)};

  despot::Belief *beliefCp{belief->MakeCopy()};

  std::vector<despot::State *> particlesCp{beliefCp->Sample(1)};

  CoverageState *coverState{static_cast<CoverageState *>(particles.at(0))};
  CoverageState *coverStateCp{static_cast<CoverageState *>(particlesCp.at(0))};
  REQUIRE(coverState->state_id == coverStateCp->state_id);
  REQUIRE(coverState->weight == coverStateCp->weight);
  REQUIRE(coverState->robotPosition == coverStateCp->robotPosition);
  REQUIRE(coverState->time == coverStateCp->time);
  REQUIRE(coverState->covered.size() == coverStateCp->covered.size());
  REQUIRE(coverState->covered == coverStateCp->covered);
  REQUIRE(coverState->map.size() == coverStateCp->map.size());
  REQUIRE(coverState->map(0, 0) == coverStateCp->map(0, 0));
  REQUIRE(coverState->map(0, 1) == coverStateCp->map(0, 1));

  // deallocate everything
  for (despot::State *state : particles) {
    pomdp->Free(state);
  }
  for (despot::State *state : particlesCp) {
    pomdp->Free(state);
  }
  delete pomdp;
  delete beliefCp; // Original belief uses smart pointers, but copy doesn't
}

TEST_CASE("Tests for CoverageBelief::text", "[CoverageBelief::text]") {
  Eigen::MatrixXd entry{1, 2};
  Eigen::MatrixXd exit{1, 2};
  Eigen::MatrixXd initBelief{1, 2};
  entry(0, 0) = 1;
  entry(0, 1) = 1;
  exit(0, 0) = 0;
  exit(0, 1) = 0;
  initBelief(0, 0) = 0.7;
  initBelief(0, 1) = 0.4;
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, initBelief)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  const CoveragePOMDP *pomdp{new CoveragePOMDP{fov, imac, 5}};

  // Test copying using sampling and deterministic belief
  std::unique_ptr<CoverageBelief> belief{std::make_unique<CoverageBelief>(
      pomdp, GridCell{0, 1}, 1, std::set<GridCell>{GridCell{0, 0}},
      imac->getInitialBelief(), imac, fov)};

  REQUIRE(belief->text() == "Robot Position: (0, 1); Time: 1; Covered: 50%; "
                            "Map Belief: \n0.7 0.4 \n");

  // Deallocate everything
  delete pomdp;
}

TEST_CASE("Tests for CoverageBelief::Update", "[CoverageBelief::Update]") {
  Eigen::MatrixXd entry{1, 2};
  Eigen::MatrixXd exit{1, 2};
  Eigen::MatrixXd initBelief{1, 2};
  entry(0, 0) = 0.7;
  entry(0, 1) = 0.5;
  exit(0, 0) = 0.6;
  exit(0, 1) = 0.4;
  initBelief(0, 0) = 0;
  initBelief(0, 1) = 0.4;
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit, initBelief)};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  const CoveragePOMDP *pomdp{new CoveragePOMDP{fov, imac, 5}};

  // Test copying using sampling and deterministic belief
  std::unique_ptr<CoverageBelief> belief{std::make_unique<CoverageBelief>(
      pomdp, GridCell{0, 0}, 1, std::set<GridCell>{GridCell{0, 0}},
      imac->getInitialBelief(), imac, fov)};

  // Check update with success
  // 30 = 11110 in binary
  belief->Update(ActionHelpers::toInt(Action::right), 30);
  std::vector<despot::State *> particles{belief->Sample(5)};
  for (despot::State *state : particles) {
    CoverageState *coverState{static_cast<CoverageState *>(state)};
    REQUIRE(coverState->robotPosition == GridCell{1, 0});
    REQUIRE(coverState->time == 2);
    REQUIRE(coverState->covered.size() == 2);
    REQUIRE(coverState->covered.count(GridCell{0, 0}) == 1);
    REQUIRE(coverState->covered.count(GridCell{1, 0}) == 1);
    REQUIRE(coverState->map.size() == 2);
    REQUIRE(coverState->map(0, 0) == 1);
    REQUIRE(coverState->map(0, 1) == 0);
  }

  // Check update with failure
  belief->Update(ActionHelpers::toInt(Action::up), 6);
  std::vector<despot::State *> particlesFail{belief->Sample(5)};
  for (despot::State *state : particlesFail) {
    CoverageState *coverState{static_cast<CoverageState *>(state)};
    REQUIRE(coverState->robotPosition == GridCell{1, 0});
    REQUIRE(coverState->time == 3);
    REQUIRE(coverState->covered.size() == 2);
    REQUIRE(coverState->covered.count(GridCell{0, 0}) == 1);
    REQUIRE(coverState->covered.count(GridCell{1, 0}) == 1);
    REQUIRE(coverState->map.size() == 2);
    REQUIRE(coverState->map(0, 0) == 0);
    REQUIRE(coverState->map(0, 1) == 0);
  }

  // Check with no observations to test IMac forward step
  entry(0, 0) = 1;
  entry(0, 1) = 0.5;
  exit(0, 0) = 0;
  exit(0, 1) = 0.4;
  initBelief(0, 0) = 0;
  initBelief(0, 1) = 0.4;
  std::shared_ptr<IMac> imacTwo{
      std::make_shared<IMac>(entry, exit, initBelief)};

  const CoveragePOMDP *pomdpTwo{
      new CoveragePOMDP{std::vector<GridCell>{}, imacTwo, 5}};

  std::unique_ptr<CoverageBelief> beliefTwo{std::make_unique<CoverageBelief>(
      pomdpTwo, GridCell{1, 0}, 1, std::set<GridCell>{GridCell{1, 0}},
      imacTwo->getInitialBelief(), imacTwo, std::vector<GridCell>{})};

  beliefTwo->Update(ActionHelpers::toInt(Action::wait), 1);
  std::vector<despot::State *> particlesThree{beliefTwo->Sample(5)};
  for (despot::State *state : particlesThree) {
    CoverageState *coverState{static_cast<CoverageState *>(state)};
    REQUIRE(coverState->robotPosition == GridCell{1, 0});
    REQUIRE(coverState->time == 2);
    REQUIRE(coverState->covered.size() == 1);
    REQUIRE(coverState->covered.count(GridCell{1, 0}) == 1);
    REQUIRE(coverState->map.size() == 2);
    REQUIRE(coverState->map(0, 0) == 1);
    REQUIRE(coverState->map(0, 1) == 0);
  }

  // Deallocate everything
  for (despot::State *state : particles) {
    pomdp->Free(state);
  }
  for (despot::State *state : particlesFail) {
    pomdp->Free(state);
  }
  delete pomdp;

  for (despot::State *state : particlesThree) {
    pomdpTwo->Free(state);
  }
  delete pomdpTwo;
}