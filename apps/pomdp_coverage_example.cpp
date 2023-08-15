/**
 * Example script which runs the POMDP Coverage Planner
 *
 * @author: Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/pomdp_coverage_robot.h"
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <vector>

/**
 * Create a random IMac instance for this example.
 *
 * @return imac A shared pointer to an IMac instance
 */
std::shared_ptr<IMac> createIMac() {
  Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 3)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 3)};
  Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 3)};

  return std::make_shared<IMac>(entry, exit, init);
}

int main() {

  std::shared_ptr<IMac> imac{createIMac()};

  // Create the IMac Executor
  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  // The robot's field of view
  std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

  GridCell initPos{0, 0};
  int timeBound{13};

  // Assume true IMac model known
  std::unique_ptr<POMDPCoverageRobot> robot{
      std::make_unique<POMDPCoverageRobot>(initPos, timeBound, 3, 3, fov, exec,
                                           imac)};

  // Start example
  robot->runCoverageEpisode("/home/charlie/work/coverage-plan/data/results/"
                            "pomdpCoverageRobotExampleVisited.csv");

  // Log the map over time
  exec->logMapDynamics("/home/charlie/work/coverage-plan/data/results/"
                       "pomdpCoverageRobotExampleMap.csv");

  return 0;
}
