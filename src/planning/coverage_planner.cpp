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
  despot::Globals::config.discount =
      0.99999; // Can't set to 1 because of issue in trivial bounds. TODO: Fix
  despot::Globals::config.pruning_constant = 0.01; // TODO: Figure this out
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
 * Return name of solver to be used (here: DESPOT).
 */
std::string CoveragePlanner::ChooseSolver() { return "DESPOT"; }