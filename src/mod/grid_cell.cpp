/**
 * Implementation of functions in grid_cell.h.
 * @see grid_cell.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"

bool GridCell::outOfBounds(const int &xMin, const int &xMax, const int &yMin,
                           const int &yMax) {
  return (this->x < xMin) || (this->x >= xMax) || (this->y < yMin) ||
         (this->y >= yMax);
}