/**
 * @file coverage_belief.h
 *
 * @brief A class for representing coverage planning beliefs (using IMac).
 *
 * @author Charlie Street
 */

#ifndef COVERAGE_BELIEF_H
#define COVERAGE_BELIEF_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_belief_sampler.h"
#include <Eigen/Dense>
#include <despot/core/globals.h>
#include <despot/interface/belief.h>
#include <despot/interface/pomdp.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

/**
 * A class representing beliefs for coverage planning problems.
 *
 * In a CoverageState, the position, time, and covered locations are known.
 * But the map is uncertain. This class captures the belief using IMac.
 *
 * Members:
 * * _robotPosititon: The robot's position
 * * _time: The current time
 * * _covered: The locations covered by the robot
 * * _mapBelief: A distribution over the occupancy map
 * * _imac: The IMac model
 * * _fov: The robot's FOV represented as a vector of GridCells relative to the
 * * robot's position
 * * _beliefSampler: A pointer to an IMacBeliefSampler object required for
 * sampling
 */
class CoverageBelief : public despot::Belief {

private:
  GridCell _robotPosition{};
  int _time{};
  std::set<GridCell> _covered{};
  Eigen::MatrixXd _mapBelief{};
  std::shared_ptr<IMac> _imac{};
  const std::vector<GridCell> _fov{};
  std::unique_ptr<IMacBeliefSampler> _beliefSampler{};

public:
  /**
   * Initialise all attributes (call superclass constructor with nullptr).
   *
   * @param model The POMDP model containing the memory pool
   * @param initPos The robot's initial position
   * @param initTime The initial time
   * @param initCovered The initially covered vertices
   * @param initBelief The initial map belief
   * @param imac The IMac model used for planning
   * @param fov The robot's FOV as a vector of relative grid cells
   */
  CoverageBelief(const despot::DSPOMDP *model, const GridCell &initPos,
                 const int &initTime, const std::set<GridCell> &initCovered,
                 const Eigen::MatrixXd &initBelief, std::shared_ptr<IMac> imac,
                 const std::vector<GridCell> &fov)
      : Belief{model}, _robotPosition{initPos}, _time{initTime},
        _covered{initCovered}, _mapBelief{initBelief}, _imac{imac}, _fov{fov},
        _beliefSampler{std::make_unique<IMacBeliefSampler>()} {}

  ~CoverageBelief() {}

  /**
   * Sample a number of states from the IMac model.
   *
   * @param num Number of states to be sampled
   *
   * @returns a vector of sampled states
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
   * @returns The belief as a string
   */
  std::string text() const;

  /**
   * Make a copy of the belief.
   *
   * @returns A copy of this
   */
  despot::Belief *MakeCopy() const;

  /**
   * Return the occupancy map belief.
   *
   * @returns The occupancy map belief
   */
  Eigen::MatrixXd getMapBelief() const;
};

#endif