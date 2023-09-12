/**
 * Implementation of CoveragePlanner class.
 * @see coverage_planner.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_planner.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/planning/coverage_world.h"
#include <cmath>
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <despot/util/optionparser.h>
#include <despot/util/util.h>
#include <string>

/**
 * Create, initialize, and return a CoveragePOMDP model.
 */
despot::DSPOMDP *
CoveragePlanner::InitializeModel(despot::option::Option *options) {
  return new CoveragePOMDP(this->_fov, this->_planIMac, this->_timeBound);
}

/**
 * Create, initialize, and return the CoverageWorld.
 */
despot::World *
CoveragePlanner::InitializeWorld(std::string &world_type,
                                 despot::DSPOMDP *model,
                                 despot::option::Option *options) {
  CoverageWorld *world{new CoverageWorld(this->_initPos, this->_initTime,
                                         this->_timeBound, this->_fov,
                                         this->_exec)};
  world->Connect();
  world->Initialize();
  return world;
}

/**
 * Provide default values for global parameters (such as those in
 * Globals::config)
 */
void CoveragePlanner::InitializeDefaultParameters() {
  despot::Globals::config.time_per_move = 1; // 1 second of planning each step
  despot::Globals::config.sim_len = this->_timeBound + 1; // Max out
  // num_scenarios left at default TODO: Figure this out
  despot::Globals::config.search_depth = this->_timeBound + 1;       // Max out
  despot::Globals::config.max_policy_sim_len = this->_timeBound + 1; // Max out
  despot::Globals::config.discount = 1.0; // We have a finite horizon problem
  if (this->_boundType == "TRIVIAL") {
    // If trivial bounds, can't set discount factor to 1 since divide by 0
    despot::Globals::config.discount = 0.99999;
  }
  despot::Globals::config.pruning_constant = this->_pruningConstant;
  // xi left at the default for now TODO: Figure this out
  // root_seed calculation copied from Ricardo's PR and plannerbase.cpp
  long millis = (long)(despot::get_time_second() * 1000);
  long range = (long)pow((double)10, (int)9);
  despot::Globals::config.root_seed =
      (unsigned int)(millis - (millis / range) * range);
  // default_action only used with POMDPX models, can ignore
  // noise only used with POMDPX models, can ignore
  despot::Globals::config.silence = false;
}

/**
 * A wrapper around InitializeParamers which feeds in custom argv.
 */
despot::option::Option *CoveragePlanner::InitializeParameters(
    std::string &solver_type, bool &search_solver, int &num_runs,
    std::string &simulator_type, std::string &belief_type, int &time_limit) {

  // Make custom argv (lots of annoying type conversions here)
  char *argv[9];
  std::string dummyVal{"DUMMY"};
  std::string lFlag{"-l"};
  std::string uFlag{"-u"};
  std::string blFlag{"--blbtype"};
  std::string buFlag{"--bubtype"};
  argv[0] = &*dummyVal.begin();
  argv[1] = &*lFlag.begin();
  argv[2] = &*this->_boundType.begin();
  argv[3] = &*uFlag.begin();
  argv[4] = &*this->_boundType.begin();
  argv[5] = &*blFlag.begin();
  argv[6] = &*this->_boundType.begin();
  argv[7] = &*buFlag.begin();
  argv[8] = &*this->_boundType.begin();

  return this->InitializeParamers(9, argv, solver_type, search_solver, num_runs,
                                  simulator_type, belief_type, time_limit);
}

/**
 * Return name of solver to be used (here: DESPOT).
 */
std::string CoveragePlanner::ChooseSolver() { return "DESPOT"; }