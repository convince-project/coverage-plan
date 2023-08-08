/**
 * Unit tests for CoveragePOMDP in coverage_pomdp.h.
 * @see coverage_pomdp.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <despot/interface/default_policy.h>
#include <memory>
#include <sstream>
#include <vector>

// TODO: Step

TEST_CASE("Test for CoveragePOMDP::NumActions", "[CoveragePOMDP::NumActions]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  REQUIRE(pomdp->NumActions() == 5);
}

TEST_CASE("Test for CoveragePOMDP::ObsProb", "[CoveragePOMDP::ObsProb]") {

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(fov, nullptr, 5)};

  Eigen::MatrixXi map{3, 3};
  map(0, 0) = 1;
  map(0, 1) = 0;
  map(0, 2) = 1;
  map(1, 0) = 0;
  map(1, 1) = 0;
  map(1, 2) = 1;
  map(2, 0) = 0;
  map(2, 1) = 1;
  map(2, 2) = 0;

  std::vector<GridCell> covered{};

  // Test 1: Out of bounds good
  CoverageState state{GridCell{0, 1}, 0, map, covered, 1, -1};

  REQUIRE(pomdp->ObsProb(18, state, ActionHelpers::toInt(Action::wait)) == 1.0);
  REQUIRE(pomdp->ObsProb(8, state, ActionHelpers::toInt(Action::wait)) == 1.0);

  // Test 2: Out of bounds bad
  REQUIRE(pomdp->ObsProb(26, state, ActionHelpers::toInt(Action::wait)) == 0.0);
  REQUIRE(pomdp->ObsProb(10, state, ActionHelpers::toInt(Action::wait)) == 0.0);

  // Test 3: In bounds bad
  CoverageState stateTwo{GridCell{1, 1}, 0, map, covered, 1, -1};
  REQUIRE(pomdp->ObsProb(4, stateTwo, ActionHelpers::toInt(Action::wait)) ==
          0.0);
  REQUIRE(pomdp->ObsProb(20, stateTwo, ActionHelpers::toInt(Action::wait)) ==
          0.0);

  // Test 4: In bounds good
  REQUIRE(pomdp->ObsProb(5, stateTwo, ActionHelpers::toInt(Action::wait)) ==
          1.0);
  REQUIRE(pomdp->ObsProb(21, stateTwo, ActionHelpers::toInt(Action::wait)) ==
          1.0);
}

TEST_CASE("Test for CoveragePOMDP::InitialBelief",
          "[CoveragePOMDP::InitialBelief]") {
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

  Eigen::MatrixXi map{3, 3};
  map(0, 0) = 1;
  map(0, 1) = 0;
  map(0, 2) = 1;
  map(1, 0) = 1;
  map(1, 1) = 0;
  map(1, 2) = 1;
  map(2, 0) = 0;
  map(2, 1) = 0;
  map(2, 2) = 0;
  CoverageState coverState{
      GridCell{1, 1}, 2, map, std::vector<GridCell>{GridCell{1, 1}}, 0.2, -1};

  despot::Belief *belief{pomdp->InitialBelief(&coverState)};
  CoverageBelief *coverBelief{static_cast<CoverageBelief *>(belief)};

  std::vector<despot::State *> particles{coverBelief->Sample(1)};
  REQUIRE(particles.size() == 1);
  CoverageState *sample{static_cast<CoverageState *>(particles.at(0))};

  REQUIRE(sample->robotPosition == GridCell{1, 1});
  REQUIRE(sample->time == 2);
  REQUIRE(sample->covered.size() == 1);
  REQUIRE(sample->covered.at(0) == GridCell{1, 1});
  REQUIRE(sample->weight == 1.0 / 1.0);
  REQUIRE(sample->state_id == -1);
  REQUIRE(sample->map.size() == 9);
  REQUIRE(sample->map(0, 1) == 0);
  REQUIRE(sample->map(1, 0) == 1);
  REQUIRE(sample->map(1, 1) == 0);
  REQUIRE(sample->map(1, 2) == 1);
  REQUIRE(sample->map(2, 1) == 0);

  // Deallocate the belief
  delete sample;
  delete coverBelief;
}

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

TEST_CASE("Tests for CoveragePOMDP::PrintState",
          "[CoveragePOMDP::PrintState]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  CoverageState state{GridCell{1, 1}, 3, Eigen::MatrixXi::Zero(2, 2),
                      std::vector<GridCell>{GridCell{0, 0}, GridCell{0, 1},
                                            GridCell{1, 1}, GridCell{1, 2}},
                      0.5};

  // Same as test in CoverageState but through CoveragePOMDP
  std::ostringstream stream{};
  pomdp->PrintState(state, stream);

  std::string expected{"Time: 3; Coverage: 100%\n\x1b[1;32m- \033[1;0m- "
                       "\n\x1b[1;32m- \x1b[1;32mR \n\033[1;0m"};

  REQUIRE(stream.str() == expected);
}

TEST_CASE("Test for CoveragePOMDP::PrintObs", "[CoveragePOMDP::PrintObs]") {
  std::unique_ptr<CoveragePOMDP> pomdp{std::make_unique<CoveragePOMDP>(
      std::vector<GridCell>{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}},
      nullptr, 5)};

  Eigen::MatrixXd map{Eigen::MatrixXd::Zero(3, 3)};
  CoverageState state{GridCell{1, 1}, 2, map, std::vector<GridCell>{}, 0.5, -1};

  std::ostringstream stream{};

  // Test with action success
  pomdp->PrintObs(state, 26, stream);
  REQUIRE(stream.str() ==
          "Action Successful; Observation:\n? X ? \nX R - \n? - ? \n");

  // Test with action failure
  stream.clear();
  pomdp->PrintObs(state, 10, stream);
  REQUIRE(stream.str() ==
          "Action Failed; Observation:\n? X ? \nX R - \n? - ? \n");
}

TEST_CASE("Test for CoveragePOMDP::PrintAction",
          "[CoveragePOMDP::PrintAction]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  std::ostringstream stream{};

  pomdp->PrintAction(ActionHelpers::toInt(Action::up), stream);
  REQUIRE(stream.str() == "Action: Up\n");
  stream.clear();

  pomdp->PrintAction(ActionHelpers::toInt(Action::down), stream);
  REQUIRE(stream.str() == "Action: Down\n");
  stream.clear();

  pomdp->PrintAction(ActionHelpers::toInt(Action::left), stream);
  REQUIRE(stream.str() == "Action: Left\n");
  stream.clear();

  pomdp->PrintAction(ActionHelpers::toInt(Action::right), stream);
  REQUIRE(stream.str() == "Action: Right\n");
  stream.clear();

  pomdp->PrintAction(ActionHelpers::toInt(Action::wait), stream);
  REQUIRE(stream.str() == "Action: Wait\n");
  stream.clear();
}

TEST_CASE("Test for CoveragePOMDP::PrintBelief",
          "[CoveragePOMDP::PrintBelief]") {
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
      pomdp, GridCell{0, 1}, 1, std::vector<GridCell>{GridCell{0, 0}},
      imac->getInitialBelief(), imac, fov)};

  // Same as test in CoverageBelief but through CoveragePOMDP
  std::ostringstream stream{};
  pomdp->PrintBelief(*belief, stream);

  REQUIRE(stream.str() == "Robot Position: (0, 1); Time: 1; Covered: 50%; "
                          "Map Belief: \n0.7 0.4 \n");

  // Deallocate everything
  delete pomdp;
}

TEST_CASE("Test for CoveragePOMDP::Allocate/Free",
          "[CoveragePOMDP::Allocate/Free]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  despot::State *state{pomdp->Allocate(5, 0.2)};
  REQUIRE(state->state_id == 5);
  REQUIRE(state->weight == 0.2);
  REQUIRE(pomdp->NumActiveParticles() == 1);

  pomdp->Free(state);

  REQUIRE(pomdp->NumActiveParticles() == 0);
}

TEST_CASE("Test for CoveragePOMDP::Copy", "[CoveragePOMDP::Copy]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  despot::State *state{pomdp->Allocate(5, 0.2)};
  CoverageState *coverState{static_cast<CoverageState *>(state)};
  coverState->map = Eigen::MatrixXd::Zero(2, 2);
  coverState->robotPosition = GridCell{2, 1};
  coverState->time = 3;
  coverState->covered = std::vector<GridCell>{GridCell{2, 1}};

  despot::State *stateTwo{pomdp->Copy(coverState)};
  CoverageState *coverStateTwo{static_cast<CoverageState *>(stateTwo)};

  REQUIRE(coverState->weight == coverStateTwo->weight);
  REQUIRE(coverState->state_id == coverStateTwo->state_id);
  REQUIRE(coverState->robotPosition == coverStateTwo->robotPosition);
  REQUIRE(coverState->time == coverStateTwo->time);
  REQUIRE(coverState->covered.size() == coverStateTwo->covered.size());
  REQUIRE(coverState->covered.at(0) == coverStateTwo->covered.at(0));
  coverStateTwo->time = 4;
  REQUIRE(coverState->time != coverStateTwo->time);

  // Deallocate everything
  pomdp->Free(coverState);
  pomdp->Free(coverStateTwo);
}

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
