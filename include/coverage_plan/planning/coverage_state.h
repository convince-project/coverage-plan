/**
 * Header file for the CoverageState class.
 * CoverageState has the robot's location, time, covered nodes, and current map
 * state.
 *
 * @author Charlie Street
 */
#ifndef COVERAGE_STATE_H
#define COVERAGE_STATE_H

#include "coverage_plan/mod/grid_cell.h"
#include <Eigen/Dense>
#include <despot/interface/pomdp.h>

/**
 * State of Coverage POMDP.
 * Contains robot position, the time, the map, and the covered cells
 */
class CoverageState : public despot::State {

  // Following conventions in DESPOT and making fields public
public:
  GridCell robot_position;       // The robot's current grid position
  int time;                      // The current time step
  Eigen::MatrixXi map;           // The current map state
  std::vector<GridCell> covered; // The vector of covered nodes

  // Not sure how necessary this is, but following conventions of DESPOT
  // tutorial
  CoverageState() {}
  ~CoverageState() {}

  /**
   * Constructor initialises fields.
   *
   * @param curPosition The robot's current position
   * @param curTime The current time step
   * @param curMap The curent state of the map
   * @param curCovered The robot's current covered list
   */
  CoverageState(const GridCell &curPosition, const int &curTime,
                const Eigen::MatrixXi &curMap,
                const std::vector<GridCell> &curCovered)
      : robot_position{curPosition}, time{curTime}, map{curMap},
        covered{curCovered} {}
};

#endif