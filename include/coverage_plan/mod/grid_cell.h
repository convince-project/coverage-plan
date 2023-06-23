/**
 * Very small header file containing GridCell struct.
 *
 * Doesn't feel right putting it elsewhere.
 *
 * @author Charlie Street
 */
#ifndef GRID_CELL_H
#define GRID_CELL_H
#include <tuple>

/**
 * Struct for a 2D grid cell.
 *
 * Members:
 * x: The x coordinate of the cell
 * y: The y coordinate of the cell
 */
struct GridCell {
  int x{};
  int y{};

  /**
   * Implement < operator for use as key in std::map.
   *
   * @param other The GridCell to compare against
   *
   * @return lt Is this less than other?
   */
  bool operator<(const GridCell &other) const {
    return std::tie(this->x, this->y) < std::tie(other.x, other.y);
  }

  /**
   * Implement == to allow comparisons
   *
   * @param other The GridCell to compare against
   *
   * @return eq Is this eq to other?
   */
  bool operator==(const GridCell &other) const {
    return this->x == other.x && this->y == other.y;
  }
};

#endif
