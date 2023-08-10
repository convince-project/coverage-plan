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

  /**
   * Implement != to allow comparisons
   *
   * @param other The GridCell to compare against
   *
   * @return eq Is this not eq to other?
   */
  bool operator!=(const GridCell &other) const {
    return this->x != other.x or this->y != other.y;
  }

  /**
   * Implement + to make field of view calaculations simpler.
   *
   * @param other The GridCell to add
   *
   * @return summed The summed grid cell.
   */
  GridCell operator+(const GridCell &other) const {
    return GridCell{this->x + other.x, this->y + other.y};
  }

  /**
   * Function to check if a grid cell is out of bounds on a given map.
   * The bounds are defined as [xMin, xMax) and [yMin, yMax] (i.e. max is
   * exclusive).
   *
   * @param xMin The min x value
   * @param xMax The max x value
   * @param yMin The min y value
   * @param yMax The max y value
   */
  bool outOfBounds(const int &xMin, const int &xMax, const int &yMin,
                   const int &yMax) const;
};

#endif
