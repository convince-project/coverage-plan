/**
 * Tests for action-based functions in coverage_plan/planning/action.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/action.h"
#include <catch2/catch.hpp>

TEST_CASE("Tests for ActionHelpers::applySuccessfulAction function",
          "[ActionHelpers::applySuccessfulAction]") {

  GridCell cell{0, 0};

  GridCell upCell{ActionHelpers::applySuccessfulAction(cell, Action::up)};
  REQUIRE((upCell.x == 0 && upCell.y == -1));

  GridCell downCell{ActionHelpers::applySuccessfulAction(cell, Action::down)};
  REQUIRE((downCell.x == 0 && downCell.y == 1));

  GridCell leftCell{ActionHelpers::applySuccessfulAction(cell, Action::left)};
  REQUIRE((leftCell.x == -1 && leftCell.y == 0));

  GridCell rightCell{ActionHelpers::applySuccessfulAction(cell, Action::right)};
  REQUIRE((rightCell.x == 1 && rightCell.y == 0));

  GridCell waitCell{ActionHelpers::applySuccessfulAction(cell, Action::wait)};
  REQUIRE((waitCell.x == 0 && waitCell.y == 0));
}

TEST_CASE("Tests for ActionHelpers::toInt function", "[ActionHelpers::toInt]") {
  REQUIRE(ActionHelpers::toInt(Action::up) == 0);
  REQUIRE(ActionHelpers::toInt(Action::down) == 1);
  REQUIRE(ActionHelpers::toInt(Action::left) == 2);
  REQUIRE(ActionHelpers::toInt(Action::right) == 3);
  REQUIRE(ActionHelpers::toInt(Action::wait) == 4);
}

TEST_CASE("Tests for ActionHelpers::fromInt function",
          "[ActionHelpers::fromInt]") {
  REQUIRE(ActionHelpers::fromInt(0) == Action::up);
  REQUIRE(ActionHelpers::fromInt(1) == Action::down);
  REQUIRE(ActionHelpers::fromInt(2) == Action::left);
  REQUIRE(ActionHelpers::fromInt(3) == Action::right);
  REQUIRE(ActionHelpers::fromInt(4) == Action::wait);
  REQUIRE_THROWS(ActionHelpers::fromInt(-1));
  REQUIRE_THROWS(ActionHelpers::fromInt(5));
}