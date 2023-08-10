/**
 * CoverageWorld class, which wraps IMacExecutor and a bit of bookeeping.
 * Used for using IMac simulations with DESPOT.
 *
 * @author Charlie Street
 */

#ifndef COVERAGE_WORLD_H
#define COVERAGE_WORLD_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/coverage_state.h"
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/world.h>
#include <memory>
#include <vector>

/**
 * Class for coverage worlds represented using IMac.
 *
 * Attributes:
 * As in superclass, plus:
 * _initPos: Initial robot position
 * _initTime: The initial time the robot starts coverage
 * _timeBound: The time bound on coverage planning
 * _exec: An IMacExecutor which we sample through
 */
class CoverageWorld : public despot::World {

private:
  const GridCell _initPos{};
  const int _initTime{};
  const int _timeBound{};
  const std::vector<GridCell> _fov{};
  std::shared_ptr<IMacExecutor> _exec{};

public:
  /** Initialises attributes.
   *
   * @param initPos The initial position of the robot
   * @param initTime The time the robot starts coverage
   * @param timeBound The time bound on coverage planning
   * @param fov The robot's FOV as a list of GridCells relative to its position
   * @param exec The IMac executor instance we are sampling with
   */
  CoverageWorld(const GridCell &initPos, const int &initTime,
                const int &timeBound, const std::vector<GridCell> &fov,
                std::shared_ptr<IMacExecutor> exec)
      : World{}, _initPos{initPos}, _initTime{initTime},
        _timeBound{timeBound}, _fov{fov}, _exec{exec} {
    this->state_ = new CoverageState();
  }

  /**
   * Deallocates state_ (attribute in superclass, can't use smart pointers :()
   */
  ~CoverageWorld() { delete this->state_; }

  /**
   * No 'connection' needed here, just returns true.
   *
   * @return success Always true
   */
  bool Connect();

  /**
   * Resets the IMacExecutor and returns the initial coverage state.
   *
   * @return initState The initial CoverageState
   */
  despot::State *Initialize();

  /**
   * Returns the current CoverageState of the system.
   *
   * @return currentState The current CoverageState
   */
  despot::State *GetCurrentState() const;

  /**
   * Print a state.
   *
   * @param s The state to print
   * @param out The stream to write to
   */
  void PrintState(const despot::State &s, ostream &out) const;

  /**
   * Execute action, update state, and make observation.
   *
   * @param action Action to be executed in the real-world system
   * @param obs    Observation sent back from the real-world system
   *
   * @return terminal Is the current state a terminal state?
   */
  bool ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE &obs);
};

#endif