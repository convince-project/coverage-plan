/**
 * Unit tests for the functions in coverage_observation.h
 * @see coverage_observation.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_observation.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <despot/core/globals.h>
#include <tuple>

TEST_CASE("Unit test for Observation::fromObsType",
          "[Observation::fromObsType]") {

  // Test with successful action
  // 369 = 101110001 in binary
  despot::OBS_TYPE obsInt{369};
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{-1, 0}, GridCell{-1, 1},
                            GridCell{0, -1},  GridCell{0, 1},  GridCell{1, -1},
                            GridCell{1, 0},   GridCell{1, 1}};

  std::pair<std::vector<IMacObservation>, bool> obs{
      Observation::fromObsType(obsInt, fov)};

  std::vector<IMacObservation> obsVec{std::get<0>(obs)};

  REQUIRE(obsVec.size() == 8);
  REQUIRE(obsVec.at(0).cell.x == -1);
  REQUIRE(obsVec.at(0).cell.y == -1);
  REQUIRE(obsVec.at(0).occupied == 0);
  REQUIRE(obsVec.at(1).cell.x == -1);
  REQUIRE(obsVec.at(1).cell.y == 0);
  REQUIRE(obsVec.at(1).occupied == 1);
  REQUIRE(obsVec.at(2).cell.x == -1);
  REQUIRE(obsVec.at(2).cell.y == 1);
  REQUIRE(obsVec.at(2).occupied == 1);
  REQUIRE(obsVec.at(3).cell.x == 0);
  REQUIRE(obsVec.at(3).cell.y == -1);
  REQUIRE(obsVec.at(3).occupied == 1);
  REQUIRE(obsVec.at(4).cell.x == 0);
  REQUIRE(obsVec.at(4).cell.y == 1);
  REQUIRE(obsVec.at(4).occupied == 0);
  REQUIRE(obsVec.at(5).cell.x == 1);
  REQUIRE(obsVec.at(5).cell.y == -1);
  REQUIRE(obsVec.at(5).occupied == 0);
  REQUIRE(obsVec.at(6).cell.x == 1);
  REQUIRE(obsVec.at(6).cell.y == 0);
  REQUIRE(obsVec.at(6).occupied == 0);
  REQUIRE(obsVec.at(7).cell.x == 1);
  REQUIRE(obsVec.at(7).cell.y == 1);
  REQUIRE(obsVec.at(7).occupied == 1);

  REQUIRE(std::get<1>(obs));

  // Test with successful action and absolute position
  obs = Observation::fromObsType(obsInt, fov, GridCell{2, 3});

  obsVec = std::get<0>(obs);

  REQUIRE(obsVec.size() == 8);
  REQUIRE(obsVec.at(0).cell.x == 1);
  REQUIRE(obsVec.at(0).cell.y == 2);
  REQUIRE(obsVec.at(0).occupied == 0);
  REQUIRE(obsVec.at(1).cell.x == 1);
  REQUIRE(obsVec.at(1).cell.y == 3);
  REQUIRE(obsVec.at(1).occupied == 1);
  REQUIRE(obsVec.at(2).cell.x == 1);
  REQUIRE(obsVec.at(2).cell.y == 4);
  REQUIRE(obsVec.at(2).occupied == 1);
  REQUIRE(obsVec.at(3).cell.x == 2);
  REQUIRE(obsVec.at(3).cell.y == 2);
  REQUIRE(obsVec.at(3).occupied == 1);
  REQUIRE(obsVec.at(4).cell.x == 2);
  REQUIRE(obsVec.at(4).cell.y == 4);
  REQUIRE(obsVec.at(4).occupied == 0);
  REQUIRE(obsVec.at(5).cell.x == 3);
  REQUIRE(obsVec.at(5).cell.y == 2);
  REQUIRE(obsVec.at(5).occupied == 0);
  REQUIRE(obsVec.at(6).cell.x == 3);
  REQUIRE(obsVec.at(6).cell.y == 3);
  REQUIRE(obsVec.at(6).occupied == 0);
  REQUIRE(obsVec.at(7).cell.x == 3);
  REQUIRE(obsVec.at(7).cell.y == 4);
  REQUIRE(obsVec.at(7).occupied == 1);

  REQUIRE(std::get<1>(obs));

  // Test with failed action
  // 5 = 0101 in binary
  obsInt = 5;
  fov.clear();
  fov.push_back(GridCell{1, 2});
  fov.push_back(GridCell{3, 4});
  fov.push_back(GridCell{5, 6});
  obs = Observation::fromObsType(obsInt, fov);

  obsVec = std::get<0>(obs);
  REQUIRE(obsVec.size() == 3);
  REQUIRE(obsVec.at(0).cell.x == 1);
  REQUIRE(obsVec.at(0).cell.y == 2);
  REQUIRE(obsVec.at(0).occupied == 1);
  REQUIRE(obsVec.at(1).cell.x == 3);
  REQUIRE(obsVec.at(1).cell.y == 4);
  REQUIRE(obsVec.at(1).occupied == 0);
  REQUIRE(obsVec.at(2).cell.x == 5);
  REQUIRE(obsVec.at(2).cell.y == 6);
  REQUIRE(obsVec.at(2).occupied == 1);

  REQUIRE(!std::get<1>(obs));

  // Check exception case
  fov.clear();
  for (int i{0}; i < 64; ++i) {
    fov.push_back(GridCell{i, i});
  }
  obsInt = 356;

  REQUIRE_THROWS(Observation::fromObsType(obsInt, fov));
}

TEST_CASE("Unit test for Observation::toObsType", "[Observation::toObsType]") {
  // Check with action success
  std::vector<IMacObservation> obsVector{
      IMacObservation{GridCell{-1, -1}, 0}, IMacObservation{GridCell{-1, 0}, 1},
      IMacObservation{GridCell{-1, 1}, 0},  IMacObservation{GridCell{0, -1}, 0},
      IMacObservation{GridCell{0, 1}, 1},   IMacObservation{GridCell{1, -1}, 0},
      IMacObservation{GridCell{1, 0}, 1},   IMacObservation{GridCell{1, 1}, 1}};

  ActionOutcome outcome{Action::up, true, GridCell{1, 1}};

  // Binary string is 101001011 which is 331 in decimal
  REQUIRE(Observation::toObsType(obsVector, outcome) == 331);

  // Check with action failure
  obsVector.clear();
  obsVector.push_back(IMacObservation{GridCell{1, 2}, 1});
  obsVector.push_back(IMacObservation{GridCell{3, 4}, 0});
  obsVector.push_back(IMacObservation{GridCell{5, 6}, 0});

  outcome = ActionOutcome{Action::up, false, GridCell{1, 1}};

  // Binary string is 0100 which is 12 in decimal
  REQUIRE(Observation::toObsType(obsVector, outcome) == 4);

  // Check exception case
  obsVector.clear();
  for (int i{0}; i < 64; ++i) {
    obsVector.push_back(IMacObservation{GridCell{i, i}, 1});
  }
  outcome = ActionOutcome{Action::up, true, GridCell{1, 1}};

  REQUIRE_THROWS(Observation::toObsType(obsVector, outcome));
}

TEST_CASE("Unit test for Observation::computeObservation",
          "[Observation::computeObservation]") {

  Eigen::MatrixXi map{3, 3};
  map(0, 0) = 1;
  map(0, 1) = 0;
  map(0, 2) = 1;
  map(1, 0) = 0;
  map(1, 1) = 0;
  map(1, 2) = 1;
  map(2, 0) = 1;
  map(2, 1) = 1;
  map(2, 2) = 2;

  GridCell robotPos{1, 1};
  ActionOutcome outcome{Action::right, true, robotPos};

  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  despot::OBS_TYPE obs{
      Observation::computeObservation(map, robotPos, outcome, fov)};
  REQUIRE(obs == 21);

  outcome.success = false;
  obs = Observation::computeObservation(map, robotPos, outcome, fov);
  REQUIRE(obs == 5);
}