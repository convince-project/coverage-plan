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
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/world.h>
#include <memory>

/**
 * Class for coverage worlds represented using IMac.
 *
 * Attributes:
 * As in superclass, plus:
 * _initPos: Initial robot position
 * _initTime: The initial time the robot starts coverage
 * _exec: An IMacExecutor which we sample through
 */
class CoverageWorld : public despot::World {

private:
  const GridCell _initPos{};
  const int _initTime{};
  std::unique_ptr<IMacExecutor> _exec{};

public:
  /** Initialises attributes.
   *
   * @param initPos The initial position of the robot
   * @param initTime The time the robot starts coverage
   * @param imac The imac instance we are going to sample through
   */
  CoverageWorld(const GridCell &initPos, const int &initTime,
                std::shared_ptr<IMac> imac)
      : World{}, _initPos{initPos}, _initTime{initTime},
        _exec{std::make_unique<IMacExecutor>(imac)} {}

  /**
   * No 'connection' needed here, just returns true.
   *
   * @return success Always true
   */
  virtual bool Connect();

  /**
   * Resets the IMacExecutor and returns the initial coverage state.
   *
   * @return initState The initial CoverageState
   */
  virtual despot::State *Initialize(); // TODO: Add init loc as observation!

  /**
   * Returns the current CoverageState of the system.
   *
   * @return currentState The current CoverageState
   */
  virtual despot::State *GetCurrentState() const;

  /**
   * Print a state.
   *
   * @param s The state to print
   * @param out The stream to write to
   */
  virtual void PrintState(const despot::State &s, ostream &out) const;

  /**
   * Execute action, update state, and make observation.
   *
   * @param action Action to be executed in the real-world system
   * @param obs    Observation sent back from the real-world system
   *
   * @return terminal Is the current state a terminal state?
   */
  virtual bool ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE &obs);
};

#endif