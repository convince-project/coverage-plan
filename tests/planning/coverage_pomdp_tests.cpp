/**
 * Unit tests for CoveragePOMDP in coverage_pomdp.h.
 * @see coverage_pomdp.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <despot/interface/default_policy.h>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <vector>

TEST_CASE("Test for CoveragePOMDP::Step", "[CoveragePOMDP::Step]") {

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

  CoverageState state{GridCell{1, 1}, 4, Eigen::MatrixXi::Zero(3, 3),
                      std::set<GridCell>{}, 1.0};
  despot::ACT_TYPE action{ActionHelpers::toInt(Action::right)};
  despot::OBS_TYPE obs{0};
  double reward{0.0};

  // Should return terminal state (time bound met)
  // Also checks time, reward gain, covered addition
  REQUIRE(pomdp->Step(state, 0.5, action, reward, obs));
  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, fov, state.robotPosition)};
  REQUIRE(reward == 0.0);
  REQUIRE(state.time == 5);
  REQUIRE(state.covered.size() == 1);
  if (state.robotPosition == GridCell{1, 1}) { // Action fail
    REQUIRE(state.map(1, 1) == 0);
    REQUIRE(state.covered.count(GridCell{1, 1}) == 1);
    REQUIRE(!std::get<1>(obsInfo));
  } else if (state.robotPosition == GridCell{2, 1}) { // Action success
    REQUIRE(state.map(1, 2) == 0);
    REQUIRE(state.covered.count(GridCell{2, 1}) == 1);
    REQUIRE(std::get<1>(obsInfo));
  } else {
    REQUIRE(false);
  }

  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    if (!imacObs.cell.outOfBounds(0, state.map.cols(), 0, state.map.rows())) {
      REQUIRE(state.map(imacObs.cell.y, imacObs.cell.x) == imacObs.occupied);
    } else {
      REQUIRE(imacObs.occupied == 1);
    }
  }

  // terminal state (all nodes covered)
  std::set<GridCell> covered{};
  for (int x{0}; x < 3; ++x) {
    for (int y{0}; y < 3; ++y) {
      covered.insert(GridCell{x, y});
    }
  }
  state = CoverageState{GridCell{1, 1}, 3, Eigen::MatrixXi::Zero(3, 3), covered,
                        1.0};
  REQUIRE(pomdp->Step(state, 0.4, action, reward, obs));
  obsInfo = Observation::fromObsType(obs, fov, state.robotPosition);
  REQUIRE(reward == 0.0);
  REQUIRE(state.covered.size() == 9);
  REQUIRE(state.time == 4);
  if (state.robotPosition == GridCell{1, 1}) { // Action fail
    REQUIRE(state.map(1, 1) == 0);
    REQUIRE(state.covered.count(GridCell{1, 1}) == 1);
    REQUIRE(!std::get<1>(obsInfo));
  } else if (state.robotPosition == GridCell{2, 1}) { // Action success
    REQUIRE(state.map(1, 2) == 0);
    REQUIRE(state.covered.count(GridCell{2, 1}) == 1);
    REQUIRE(std::get<1>(obsInfo));
  } else {
    REQUIRE(false);
  }
  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    if (!imacObs.cell.outOfBounds(0, state.map.cols(), 0, state.map.rows())) {
      REQUIRE(state.map(imacObs.cell.y, imacObs.cell.x) == imacObs.occupied);
    } else {
      REQUIRE(imacObs.occupied == 1);
    }
  }

  // Test when not terminal
  // Wait actions always succeed
  state = CoverageState{GridCell{0, 1}, 1, Eigen::MatrixXi::Zero(3, 3),
                        std::set<GridCell>{GridCell{1, 1}}, 1.0};
  REQUIRE(!pomdp->Step(state, 0.3, ActionHelpers::toInt(Action::wait), reward,
                       obs));
  obsInfo = Observation::fromObsType(obs, fov, state.robotPosition);
  // Reward for wait should always be 0 in practice, but because I've spammed
  // the covered list with complete nonsense, the robot's waiting location isn't
  // included. As seed 0.3 leads to action success, we get a reward of 1
  // so this still demonstrates the intended behaviour
  REQUIRE(reward == 1.0);
  REQUIRE(state.robotPosition == GridCell{0, 1});
  REQUIRE(state.covered.size() == 2);
  REQUIRE(state.covered.count(GridCell{0, 1}) == 1);
  REQUIRE(state.time == 2);
  REQUIRE(std::get<1>(obsInfo));
  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    if (!imacObs.cell.outOfBounds(0, state.map.cols(), 0, state.map.rows())) {
      REQUIRE(state.map(imacObs.cell.y, imacObs.cell.x) == imacObs.occupied);
    } else {
      REQUIRE(imacObs.occupied == 1);
    }
  }
}

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

  std::set<GridCell> covered{};

  // Test 1: Out of bounds good
  CoverageState state{GridCell{0, 1}, 0, map, covered, 1, -1};

  REQUIRE(pomdp->ObsProb(26, state, ActionHelpers::toInt(Action::wait)) == 1.0);
  REQUIRE(pomdp->ObsProb(10, state, ActionHelpers::toInt(Action::wait)) == 1.0);

  // Test 2: Out of bounds bad
  REQUIRE(pomdp->ObsProb(18, state, ActionHelpers::toInt(Action::wait)) == 0.0);
  REQUIRE(pomdp->ObsProb(2, state, ActionHelpers::toInt(Action::wait)) == 0.0);

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
      GridCell{1, 1}, 2, map, std::set<GridCell>{GridCell{1, 1}}, 0.2, -1};

  despot::Belief *belief{pomdp->InitialBelief(&coverState)};
  CoverageBelief *coverBelief{static_cast<CoverageBelief *>(belief)};

  std::vector<despot::State *> particles{coverBelief->Sample(1)};
  REQUIRE(particles.size() == 1);
  CoverageState *sample{static_cast<CoverageState *>(particles.at(0))};

  REQUIRE(sample->robotPosition == GridCell{1, 1});
  REQUIRE(sample->time == 2);
  REQUIRE(sample->covered.size() == 1);
  REQUIRE(sample->covered.count(GridCell{1, 1}) == 1);
  REQUIRE(sample->weight == 1.0 / 1.0);
  REQUIRE(sample->state_id == -1);
  REQUIRE(sample->map.size() == 9);
  REQUIRE(sample->map(0, 1) == 0);
  REQUIRE(sample->map(1, 0) == 1);
  REQUIRE(sample->map(1, 1) == 0);
  REQUIRE(sample->map(1, 2) == 1);
  REQUIRE(sample->map(2, 1) == 0);

  // Deallocate the belief
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

TEST_CASE("Tests for CoveragePOMDP::CreateScenarioUpperBound",
          "[CoveragePOMDP::CreateScenarioUpperBound]") {
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
  despot::History history{};
  despot::RandomStreams streams{3, 10};

  despot::ScenarioUpperBound *bound{pomdp->CreateScenarioUpperBound()};
  CoverageState state{GridCell{0, 0}, 1, Eigen::MatrixXi::Zero(3, 3),
                      std::set<GridCell>{}, 1.0};
  REQUIRE_THAT(
      bound->Value(std::vector<despot::State *>{&state}, streams, history),
      Catch::Matchers::WithinRel(4.0, 0.001));
  delete bound;

  bound = pomdp->CreateScenarioUpperBound("MAX_CELLS");
  REQUIRE_THAT(
      bound->Value(std::vector<despot::State *>{&state}, streams, history),
      Catch::Matchers::WithinRel(4.0, 0.001));
  delete bound;

  bound = pomdp->CreateScenarioUpperBound("TRIVIAL");
  despot::Globals::config.discount = 0.5;
  REQUIRE_THAT(
      bound->Value(std::vector<despot::State *>{&state}, streams, history),
      Catch::Matchers::WithinRel(2.0, 0.001));
  delete bound;
}

TEST_CASE("Tests for CoveragePOMDP::CreateParticleLowerBound",
          "[CoveragePOMDP::CreateParticleLowerBound]") {
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

  despot::ParticleLowerBound *bound{pomdp->CreateParticleLowerBound()};
  CoverageState state{GridCell{0, 0}, 1, Eigen::MatrixXi::Zero(3, 3),
                      std::set<GridCell>{}, 1.0};
  REQUIRE_THAT(bound->Value(std::vector<despot::State *>{&state}).value,
               Catch::Matchers::WithinRel(0.0, 0.001));
  delete bound;

  bound = pomdp->CreateParticleLowerBound("ZERO");
  REQUIRE_THAT(bound->Value(std::vector<despot::State *>{&state}).value,
               Catch::Matchers::WithinRel(0.0, 0.001));
  delete bound;

  bound = pomdp->CreateParticleLowerBound("TRIVIAL");
  REQUIRE_THAT(bound->Value(std::vector<despot::State *>{&state}).value,
               Catch::Matchers::WithinRel(0.0, 0.001));
  delete bound;
}

TEST_CASE("Tests for CoveragePOMDP::CreateScenarioLowerBound",
          "[CoveragePOMDP::CreateScenarioLowerBound]") {
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

  despot::ScenarioLowerBound *bound{pomdp->CreateScenarioLowerBound()};

  std::vector<despot::State *> particles{};
  for (int i{0}; i < 5; ++i) {
    CoverageState *state{
        static_cast<CoverageState *>(pomdp->Allocate(-1, 0.2))};
    state->robotPosition = GridCell{1, 1};
    state->time = 3;
    state->map = Eigen::MatrixXi::Zero(3, 3);
    state->map(1, 0) = 1;
    state->map(2, 1) = 1;
    state->covered = std::set<GridCell>{GridCell{1, 1}, GridCell{1, 0}};
    particles.push_back(state);
  }

  despot::RandomStreams streams{3, 10};
  despot::History history{};

  despot::ValuedAction va{bound->Value(particles, streams, history)};
  REQUIRE(va.action == ActionHelpers::toInt(Action::down));
  delete bound;

  bound = pomdp->CreateScenarioLowerBound("GREEDY");
  va = bound->Value(particles, streams, history);
  REQUIRE(va.action == ActionHelpers::toInt(Action::down));
  delete bound;

  bound = pomdp->CreateScenarioLowerBound("TRIVIAL");
  va = bound->Value(particles, streams, history);
  REQUIRE(va.action == ActionHelpers::toInt(Action::up));
  delete bound;

  for (despot::State *state : particles) {
    pomdp->Free(state);
  }
}

TEST_CASE("Tests for CoveragePOMDP::PrintState",
          "[CoveragePOMDP::PrintState]") {
  std::unique_ptr<CoveragePOMDP> pomdp{
      std::make_unique<CoveragePOMDP>(std::vector<GridCell>{}, nullptr, 5)};

  CoverageState state{GridCell{1, 1}, 3, Eigen::MatrixXi::Zero(2, 2),
                      std::set<GridCell>{GridCell{0, 0}, GridCell{0, 1},
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

  Eigen::MatrixXi map{Eigen::MatrixXi::Zero(3, 3)};
  CoverageState state{GridCell{1, 1}, 2, map, std::set<GridCell>{}, 0.5, -1};

  std::ostringstream stream{};

  // Test with action success
  pomdp->PrintObs(state, 26, stream);
  REQUIRE(stream.str() ==
          "Action Successful; Observation:\n? X ? \nX R - \n? - ? \n");

  // Test with action failure
  stream = std::ostringstream{};
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
  stream = std::ostringstream{};

  pomdp->PrintAction(ActionHelpers::toInt(Action::down), stream);
  REQUIRE(stream.str() == "Action: Down\n");
  stream = std::ostringstream{};

  pomdp->PrintAction(ActionHelpers::toInt(Action::left), stream);
  REQUIRE(stream.str() == "Action: Left\n");
  stream = std::ostringstream{};

  pomdp->PrintAction(ActionHelpers::toInt(Action::right), stream);
  REQUIRE(stream.str() == "Action: Right\n");
  stream = std::ostringstream{};

  pomdp->PrintAction(ActionHelpers::toInt(Action::wait), stream);
  REQUIRE(stream.str() == "Action: Wait\n");
  stream = std::ostringstream{};
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
      pomdp, GridCell{0, 1}, 1, std::set<GridCell>{GridCell{0, 0}},
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
  coverState->map = Eigen::MatrixXi::Zero(2, 2);
  coverState->robotPosition = GridCell{2, 1};
  coverState->time = 3;
  coverState->covered = std::set<GridCell>{GridCell{2, 1}};

  despot::State *stateTwo{pomdp->Copy(coverState)};
  CoverageState *coverStateTwo{static_cast<CoverageState *>(stateTwo)};

  REQUIRE(coverState->weight == coverStateTwo->weight);
  REQUIRE(coverState->state_id == coverStateTwo->state_id);
  REQUIRE(coverState->robotPosition == coverStateTwo->robotPosition);
  REQUIRE(coverState->time == coverStateTwo->time);
  REQUIRE(coverState->covered.size() == coverStateTwo->covered.size());
  REQUIRE(coverState->covered == coverStateTwo->covered);
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
