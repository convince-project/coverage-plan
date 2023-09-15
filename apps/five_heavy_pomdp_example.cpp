/**
 * Example script which runs the POMDP coverage planner on a 5x5 map.
 * The outcome of this will then be used to produce an animation.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/fixed_imac_executor.h"
#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/planning/pomdp_coverage_robot.h"
#include <filesystem>
#include <memory>
#include <vector>

int main() {

  // Read in IMac model and executor
  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>("../../data/prelim_exps/five_heavy")};
  std::vector<std::filesystem::path> runFileVec{
      "../../data/prelim_exps/five_heavy/run_1.csv"};
  int xDim{5};
  int yDim{5};
  std::shared_ptr<FixedIMacExecutor> exec{
      std::make_shared<FixedIMacExecutor>(runFileVec, xDim, yDim)};

  // Robot FOV
  std::vector<GridCell> fov{GridCell{-1, -1}, GridCell{0, -1}, GridCell{1, -1},
                            GridCell{-1, 0},  GridCell{1, 0},  GridCell{-1, 1},
                            GridCell{0, 1},   GridCell{1, 1}};

  // Other params
  GridCell initPos{0, 0};
  int timeBound{40};

  std::unique_ptr<POMDPCoverageRobot> robot{
      std::make_unique<POMDPCoverageRobot>(initPos, timeBound, xDim, yDim, fov,
                                           exec, imac)};

  // Start example
  robot->runCoverageEpisode("/home/charlie/work/coverage-plan/data/results/"
                            "prelim_exps/fiveHeavyExampleVisited.csv");

  // Log the map dynamics
  exec->logMapDynamics(
      "/home/charlie/work/coverage-plan/data/results/fiveHeavyExampleMap.csv");

  return 0;
}