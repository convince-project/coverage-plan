/**
 * @file coverage_state.h
 *
 * @brief Header file for the CoverageState class.
 *
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
#include <set>
#include <string>

/**
 * State of Coverage POMDP.
 * Contains robot position, the time, the map, and the covered cells
 *
 * Members:
 * * robotPosition: The robot's position
 * * time: The current time
 * * map: The current map
 * * covered: The set of covered cells
 */
class CoverageState : public despot::State {

  // Following conventions in DESPOT and making fields public
public:
  GridCell robotPosition{};     // The robot's current grid position
  int time{};                   // The current time step
  Eigen::MatrixXi map{};        // The current map state
  std::set<GridCell> covered{}; // The set of covered nodes

  // If overwriting default constructor, you should also overwrite the
  // destructor iirc
  CoverageState() : despot::State{} {}
  ~CoverageState() {}

  /**
   * Constructor initialises fields.
   *
   * @param curPosition The robot's current position
   * @param curTime The current time step
   * @param curMap The curent state of the map
   * @param curCovered The robot's current covered set
   * @param particleWeight The weight if the state is a particle
   * @param id The state's id (default -1)
   */
  CoverageState(const GridCell &curPosition, const int &curTime,
                const Eigen::MatrixXi &curMap,
                const std::set<GridCell> &curCovered,
                const double &particleWeight, const int &id = -1)
      : State{id, particleWeight}, robotPosition{curPosition}, time{curTime},
        map{curMap}, covered{curCovered} {}

  /**
   * Produces a string description of a state.
   *
   * @returns A string descibing the state.
   */
  std::string text() const;
};

#endif