/**
 * A class for representing coverage planning beliefs (using IMac).
 *
 * @author Charlie Street
 */

#ifndef COVERAGE_BELIEF_H
#define COVERAGE_BELIEF_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <despot/core/globals.h>
#include <despot/interface/belief.h>
#include <despot/interface/pomdp.h>
#include <memory>
#include <string>
#include <vector>

/**
 * A class representing beliefs for coverage planning problems.
 * In a CoverageState, the position, time, and covered locations are known.
 * But the map is uncertain. This class captures the belief using IMac.
 *
 * Members:
 * _robotPosititon: The robot's position
 * _time: The current time
 * _covered: The locations covered by the robot
 * _mapBelief: A distribution over the occupancy map
 * _imac: The IMac model
 */
class CoverageBelief : public despot::Belief {

private:
  GridCell _robotPosition{};
  int _time{};
  std::vector<GridCell> _covered{};
  Eigen::MatrixXd _mapBelief{};
  std::shared_ptr<IMac> _imac{};

public:
  /**
   * Initialise all attributes (call superclass constructor with nullptr).
   *
   * @param initPos The robot's initial position
   * @param initTime The initial time
   * @param initCovered The initially covered vertices
   * @param initBelief The initial map belief
   * @param imac The IMac model used for planning
   */
  CoverageBelief(const GridCell &initPos, const int &initTime,
                 const std::vector<GridCell> &initCovered,
                 const Eigen::MatrixXd &initBelief, std::shared_ptr<IMac> imac)
      : Belief{nullptr}, _robotPosition{initPos}, _time{initTime},
        _covered{initCovered}, _mapBelief{initBelief}, _imac{imac} {}

  ~CoverageBelief() {}

  /**
   * Sample a number of states from the IMac model.
   *
   * @param num Number of states to be sampled
   *
   * @return sampledStates a vector of sampled states
   */
  std::vector<despot::State *> Sample(int num) const;

  /**
   * Update the belief.
   * Runs a step through the IMac model, and sets the observations in the map.
   *
   * @param action The action taken in the last step
   * @param obs    The observation received in the last step
   */
  void Update(despot::ACT_TYPE action, despot::OBS_TYPE obs);

  /**
   * Convert belief into a string.
   *
   * @return beliefStr The belief as a string
   */
  std::string text() const;

  /**
   * Make a copy of the belief.
   *
   * @return beliefCpy A copy of this
   */
  despot::Belief *MakeCopy() const;
};

#endif