/**
 * Tests for the GridCell struct.
 *
 * @author Charlie Street
 */
#include "coverage_plan/mod/grid_cell.h"
#include <catch2/catch.hpp>

TEST_CASE("Tests for GridCell", "[GridCell]") {
  GridCell cell{5, 6};

  REQUIRE(cell == cell);
  REQUIRE(!(cell != cell));
  REQUIRE(!(cell < cell));

  GridCell cell2{5, 6};
  REQUIRE(cell == cell2);
  REQUIRE(!(cell != cell2));
  REQUIRE(!(cell < cell2));

  GridCell cell3{6, 4};
  REQUIRE(!(cell == cell3));
  REQUIRE(cell != cell3);
  REQUIRE(cell < cell3);
  REQUIRE(!(cell3 < cell));

  GridCell cell4{5, 4};
  REQUIRE(!(cell == cell4));
  REQUIRE(cell != cell4);
  REQUIRE(cell4 < cell);
  REQUIRE(!(cell < cell4));

  GridCell cell5{5, 7};
  REQUIRE(!(cell == cell5));
  REQUIRE(cell != cell5);
  REQUIRE(cell < cell5);
  REQUIRE(!(cell5 < cell));

  GridCell cell6{7, 7};
  REQUIRE(!(cell == cell6));
  REQUIRE(cell != cell6);
  REQUIRE(cell < cell6);
  REQUIRE(!(cell6 < cell));

  GridCell cell7{2, 0};
  REQUIRE(!(cell == cell7));
  REQUIRE(cell != cell7);
  REQUIRE(!(cell < cell7));
  REQUIRE(cell7 < cell);

  int xMin{0};
  int xMax{10};
  int yMin{-1};
  int yMax{1};

  REQUIRE(!GridCell{0, 0}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{-1, 0}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(!GridCell{9, 0}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{10, 0}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{11, 0}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(!GridCell{5, -1}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{5, -2}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(!GridCell{5, 0}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{5, 1}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{5, 2}.outOfBounds(xMin, xMax, yMin, yMax));
  REQUIRE(GridCell{20, 20}.outOfBounds(xMin, xMax, yMin, yMax));

  GridCell addOne{1, 2};
  GridCell addTwo{5, 9};

  GridCell sum{addOne + addTwo};
  REQUIRE(sum == GridCell{6, 11});
  REQUIRE(addOne == GridCell{1, 2});
  REQUIRE(addTwo == GridCell{5, 9});
}