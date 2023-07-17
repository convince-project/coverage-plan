/**
 * Tests for action-based functions in coverage_plan/planning/action.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/planning/action.h"
#include <catch2/catch.hpp>

TEST_CASE("Tests for applySuccessfulAction function",
          "[applySuccessfulAction]") {

  GridCell cell{0, 0};

  GridCell upCell{applySuccessfulAction(cell, Action::up)};
  REQUIRE((upCell.x == 0 && upCell.y == -1));

  GridCell downCell{applySuccessfulAction(cell, Action::down)};
  REQUIRE((downCell.x == 0 && downCell.y == 1));

  GridCell leftCell{applySuccessfulAction(cell, Action::left)};
  REQUIRE((leftCell.x == -1 && leftCell.y == 0));

  GridCell rightCell{applySuccessfulAction(cell, Action::right)};
  REQUIRE((rightCell.x == 1 && rightCell.y == 0));

  GridCell waitCell{applySuccessfulAction(cell, Action::wait)};
  REQUIRE((waitCell.x == 0 && waitCell.y == 0));
}