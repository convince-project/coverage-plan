/**
 * Tests for the CoverageState class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_state.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <vector>

TEST_CASE("Tests for the CoverageState constructors",
          "[CoverageState-constructors]") {

  // Default constructor should set the state_id to -1
  CoverageState state{};
  REQUIRE(state.state_id == -1);

  // Full constructor where we set everything
  Eigen::MatrixXi map{2, 2};
  map(0, 0) = 1;
  map(0, 1) = 2;
  map(1, 0) = 3;
  map(1, 1) = 4;
  CoverageState stateTwo{GridCell{1, 2}, 3, map,
                         std::vector<GridCell>{GridCell{0, 0}, GridCell{0, 1},
                                               GridCell{1, 1}, GridCell{1, 2}},
                         0.5};

  REQUIRE(stateTwo.robot_position.x == 1);
  REQUIRE(stateTwo.robot_position.y == 2);
  REQUIRE(stateTwo.time == 3);
  REQUIRE(stateTwo.map.rows() == 2);
  REQUIRE(stateTwo.map.cols() == 2);
  REQUIRE(stateTwo.map(0, 0) == 1);
  REQUIRE(stateTwo.map(0, 1) == 2);
  REQUIRE(stateTwo.map(1, 0) == 3);
  REQUIRE(stateTwo.map(1, 1) == 4);
  REQUIRE(stateTwo.covered.size() == 4);
  REQUIRE(stateTwo.covered.at(0).x == 0);
  REQUIRE(stateTwo.covered.at(0).y == 0);
  REQUIRE(stateTwo.covered.at(1).x == 0);
  REQUIRE(stateTwo.covered.at(1).y == 1);
  REQUIRE(stateTwo.covered.at(2).x == 1);
  REQUIRE(stateTwo.covered.at(2).y == 1);
  REQUIRE(stateTwo.covered.at(3).x == 1);
  REQUIRE(stateTwo.covered.at(3).y == 2);
  REQUIRE(stateTwo.state_id == -1);
  REQUIRE(stateTwo.weight == 0.5);
}

TEST_CASE("Tests for the CoverageState text function", "[CoverageState-text]") {

  CoverageState state{GridCell{1, 1}, 3, Eigen::MatrixXi::Zero(2, 2),
                      std::vector<GridCell>{GridCell{0, 0}, GridCell{0, 1},
                                            GridCell{1, 1}, GridCell{1, 2}},
                      0.5};

  std::string expected{"Time: 3; Coverage: 100%\n\x1b[1;32m- \033[1;0m- "
                       "\n\x1b[1;32m- \x1b[1;32mR \n\033[1;0m"};
  REQUIRE(state.text() == expected);
}