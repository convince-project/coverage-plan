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
  std::string _boundType{};

public:
  /**
   * Calls super constructor and initialises attributes.
   *
   * @param initPos The robot's initial position
   * @param initTime The robot's start time
   * @param timeBound The time bound on planning
   * @param fov The robot's field of view as a vector of relative grid cells
   * @param exec The IMac executor which defines the world
   * @param planIMac The IMac model used for planning
   * @param boundType The type of upper and lower bounds to use during planning
   */
  CoveragePlanner(const GridCell &initPos, const int &initTime,
                  const int &timeBound, const std::vector<GridCell> &fov,
                  std::shared_ptr<IMacExecutor> exec,
                  std::shared_ptr<IMac> planIMac,
                  std::string boundType = "DEFAULT")
      : Planner("THESE", "ARGS", "DO NOT DO", "ANYTHING"), _initPos{initPos},
        _initTime{initTime}, _timeBound{timeBound}, _fov{fov}, _exec{exec},
        _planIMac{planIMac}, _boundType{boundType} {}

  /**
   * Empty destructor.
   */
  ~CoveragePlanner() {}

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
   * A wrapper around InitializeParamers which feeds in custom argv.
   * Also fixes the typo in the function name.
   *
   * @param solver_type The type of solver
   * @param search_solver Not entirely sure. Inherited from DESPOT
   * @param num_runs How many runs to run
   * @param simulator_type The simulation type
   * @param belief_type The type of belief used for planning
   * @param time_limit The time limit for planning
   *
   * @return options The options fed into the planner
   *
   */
  despot::option::Option *
  InitializeParameters(std::string &solver_type, bool &search_solver,
                       int &num_runs, std::string &simulator_type,
                       std::string &belief_type, int &time_limit);

  /**
   * Return name of solver to be used (here: DESPOT).
   *
   * @return solverName The name of the solver
   */
  virtual std::string ChooseSolver();
};

#endif