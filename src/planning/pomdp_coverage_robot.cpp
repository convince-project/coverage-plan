/**
 * Implementation of POMDPCoverageRobot in pomdp_coverage_robot.h
 * @see pomdp_coverage_robot.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/pomdp_coverage_robot.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_planner.h"
#include "coverage_plan/planning/coverage_pomdp.h"
#include "coverage_plan/planning/coverage_state.h"
#include "coverage_plan/planning/coverage_world.h"
#include <despot/core/globals.h>
#include <despot/core/solver.h>
#include <despot/interface/pomdp.h>
#include <despot/util/optionparser.h>
#include <memory>
#include <tuple>
#include <vector>

/**
 * Synthesises an action using the POMDP planner.

 */
Action
POMDPCoverageRobot::_planFn(const GridCell &currentLoc,
                            const std::vector<Action> &enabledActions, int ts,
                            int timeBound, std::shared_ptr<IMac> imac,
                            const std::vector<GridCell> &visited,
                            const std::vector<IMacObservation> &currentObs) {
  return ActionHelpers::fromInt(this->_solver->Search().action);
}

/**
 * Executes an action using a CoverageWorld object.
 */
ActionOutcome POMDPCoverageRobot::_executeFn(const GridCell &currentLoc,
                                             const Action &action) {
  // Apply action in CoverageWorld
  despot::OBS_TYPE obs{0};
  this->_world->ExecuteAction(ActionHelpers::toInt(action), obs);

  // Reformat the observation
  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, this->_fov)};

  // Create the ActionOutcome object
  bool succ{std::get<1>(obsInfo)};
  GridCell nextLoc{currentLoc};
  if (succ) {
    nextLoc = ActionHelpers::applySuccessfulAction(currentLoc, action);
  }
  ActionOutcome outcome{action, succ, nextLoc};

  // Log current transition
  this->_printCurrentTransition(currentLoc, outcome);

  // Add observation to be picked up by makeObservations
  this->_latestObs = std::get<0>(obsInfo);

  // Now do a belief update (has to be done here as we need the action)
  this->_solver->BeliefUpdate(ActionHelpers::toInt(action), obs);
  return outcome;
}

/**
 *  Returns the latest observation obtained from the CoverageWorld.
 */
std::vector<IMacObservation>
POMDPCoverageRobot::_observeFn(const GridCell &currentLoc) {
  return this->_latestObs;
}

/**
 * Computes initial observation, which is different to others.
 */
std::vector<IMacObservation>
POMDPCoverageRobot::_initialObservation(const GridCell &startLoc) {
  std::vector<IMacObservation> initObs{};

  CoverageState *state{
      static_cast<CoverageState *>(this->_world->GetCurrentState())};

  // The action here doesn't matter
  despot::OBS_TYPE obs{Observation::computeObservation(
      state->map, startLoc, ActionOutcome{Action::wait, true, startLoc},
      this->_fov)};

  return std::get<0>(Observation::fromObsType(obs, this->_fov, startLoc));
}

/**
 * Runs necessary setup for creating the POMDP planner.
 */
void POMDPCoverageRobot::episodeSetup(const GridCell &startLoc, const int &ts,
                                      const int &timeBound,
                                      std::shared_ptr<IMac> imacForEpisode) {
  // Call superclass function
  CoverageRobot::episodeSetup(startLoc, ts, timeBound, imacForEpisode);

  // Make planner
  this->_planner = std::make_unique<CoveragePlanner>(
      startLoc, ts, timeBound, this->_fov, this->_exec, imacForEpisode,
      this->_boundType);

  // Now, use planner to set everything up
  // Largely copied from despot/planner.cpp but broken up a bit
  // Set parameters
  std::string solver_type{this->_planner->ChooseSolver()};
  bool search_solver{};
  int num_runs{1};
  std::string world_type{"DEFAULT"};
  std::string belief_type{"DEFAULT"};
  int time_limit{-1};

  despot::option::Option *options =
      this->_planner->InitializeParameters(solver_type, search_solver, num_runs,
                                           world_type, belief_type, time_limit);
  if (options == NULL) {
    exit(0);
  }

  // Create POMDP
  this->_pomdp =
      static_cast<CoveragePOMDP *>(this->_planner->InitializeModel(options));
  assert(this->_pomdp != NULL);

  // Create world
  this->_world = static_cast<CoverageWorld *>(
      this->_planner->InitializeWorld(world_type, this->_pomdp, options));
  assert(this->_world != NULL);

  // Create belief
  this->_belief = static_cast<CoverageBelief *>(this->_pomdp->InitialBelief(
      this->_world->GetCurrentState(), belief_type));
  assert(this->_belief != NULL);

  // Create solver
  this->_solver = this->_planner->InitializeSolver(this->_pomdp, this->_belief,
                                                   solver_type, options);

  // Display solver parameters
  this->_planner->DisplayParameters(options, this->_pomdp);

  // Make the initial observation
  this->_latestObs = this->_initialObservation(startLoc);
}

/**
 * Deallocates objects used by the CoveragePlanner object.
 */
void POMDPCoverageRobot::episodeCleanup() {
  // Doing the cleanup the despot authors won't do...
  this->_latestObs = std::vector<IMacObservation>{};
  if (this->_planner != nullptr) {
    this->_planner = nullptr;
  }

  if (this->_pomdp != nullptr) {
    delete this->_pomdp;
    this->_pomdp = nullptr;
  }

  if (this->_world != nullptr) {
    delete this->_world;
    this->_world = nullptr;
  }

  if (this->_belief != nullptr) {
    delete this->_belief;
    this->_belief = nullptr;
  }

  if (this->_solver != nullptr) {
    // The following three lines are about as good as I can manage re
    // memory management. If any bounds have additional bound objects within
    // them, these will sadly cause memory leaks
    despot::DESPOT *despotSolver{static_cast<despot::DESPOT *>(this->_solver)};
    delete despotSolver->lower_bound();
    delete despotSolver->upper_bound();
    delete this->_solver;
    this->_solver = nullptr;
  }
}