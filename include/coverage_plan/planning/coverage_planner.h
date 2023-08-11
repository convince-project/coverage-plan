/**
 * CoveragePlanner class which brings everything together.
 *
 * @author Charlie Street
 */

#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <despot/interface/pomdp.h>
#include <despot/planner.h>
#include <despot/util/optionparser.h>
#include <string>
#include <vector>

/**
 * Class for POMDP coverage planner
 *
 * Attributes:
 * Same as superclass, plus:
 * _initPos: The robot's initial location
 * _initTime: The robot's starting time
 * _timeBound: The time bound for planning
 * _fov: The robot's FOV as a vector of relative grid cells
 * _exec: The IMac executor describing how the *real world* operates
 * _planIMac: The IMac model used for planning, which may not match with _exec
 */
class CoveragePlanner : public despot::Planner {

private:
  const GridCell _initPos{};
  const int _initTime{};
  const int _timeBound{};
  const std::vector<GridCell> _fov{};
  std::shared_ptr<IMacExecutor> _exec{};
  std::shared_ptr<IMac> _planIMac{};

public:
  CoveragePlanner(const GridCell &initPos, const int &initTime,
                  const int &timeBound, const std::vector<GridCell> &fov,
                  std::shared_ptr<IMacExecutor> exec,
                  std::shared_ptr<IMac> planIMac,
                  string lower_bounds_str = "TRIVIAL",
                  string base_lower_bounds_str = "TRIVIAL",
                  string upper_bounds_str = "TRIVIAL",
                  string base_upper_bounds_str = "TRIVIAL")
      : Planner(lower_bounds_str, base_lower_bounds_str, upper_bounds_str,
                base_upper_bounds_str),
        _initPos{initPos}, _initTime{initTime},
        _timeBound{timeBound}, _fov{fov}, _exec{exec}, _planIMac{planIMac} {}

  /**
   * Create, initialize, and return a CoveragePOMDP model.
   *
   * @param options Parsed command line options (not used)
   *
   * @return pomdp The coverage POMDP
   */
  despot::DSPOMDP *InitializeModel(despot::option::Option *options);

  /**
   * Create, initialize, and return the CoverageWorld.
   * All args are ignored
   *
   * @param world_type Type of the world: pomdp, simulator, or real-world
   * @param model      The POMDP model
   * @param options    Parsed command line options
   *
   * @return world The coverage world
   */
  despot::World *InitializeWorld(std::string &world_type,
                                 despot::DSPOMDP *model,
                                 despot::option::Option *options);

  /**
   * Provide default values for global parameters (such as those in
   * Globals::config)
   */
  virtual void InitializeDefaultParameters();

  /**
   * Return name of solver to be used (here: DESPOT).
   *
   * @return solverName The name of the solver
   */
  virtual std::string ChooseSolver();
};

#endif