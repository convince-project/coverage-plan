/**
 * Very small header file containing GridCell struct.
 *
 * Doesn't feel right putting it elsewhere.
 *
 * @author Charlie Street
 */
#ifndef GRID_CELL_H
#define GRID_CELL_H

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
};

#endif
