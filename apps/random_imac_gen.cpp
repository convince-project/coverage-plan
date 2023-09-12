/**
 * Script for generating random IMac environments, and traces through them.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/util/seed.h"
#include <Eigen/Dense>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>

/**
 * Create a random IMac instance of a given size and dynamics.
 *
 * @param xDim X dimension of the map
 * @param yDim Y dimension of the map
 * @param propSemi Proportion of map to be semi-static
 * @param propDynamic Proportion of map to be dynamic
 *
 * 1-(propSemi+propDynamic) of the map is free
 *
 * @return imac A shared pointer to an IMac instance
 */
std::shared_ptr<IMac> createIMac(const int &xDim, const int &yDim,
                                 const double &propSemi,
                                 const double &propDynamic) {
  Eigen::MatrixXd entryMatrix{yDim, xDim};
  Eigen::MatrixXd exitMatrix{yDim, xDim};
  Eigen::MatrixXd initialBelief{yDim, xDim};

  int numSet{0};
  const int semiStaticLimit{(int)ceil(propSemi * (xDim * yDim))};
  const int dynamicLimit{
      (int)ceil(semiStaticLimit + (propDynamic * (xDim * yDim)))};

  std::cout << "(Free, Semi-Static, Dynamic): ("
            << ((xDim * yDim) - dynamicLimit) << ", " << (semiStaticLimit - 1)
            << ", " << (dynamicLimit - semiStaticLimit) << ")\n";

  // Random order of cells on grid map
  std::vector<GridCell> cells{};
  for (int x{0}; x < xDim; ++x) {
    for (int y{0}; y < yDim; ++y) {
      cells.push_back(GridCell{x, y});
    }
  }
  std::mt19937_64 rng{SeedHelpers::genRandomDeviceSeed()};
  std::shuffle(std::begin(cells), std::end(cells), rng);

  std::uniform_real_distribution<> perturb{-0.01, 0.01};

  // Have to use y,x to match coordinate systems up
  // Add some perturbations to the dynamic cells to make it more representative
  for (const GridCell &cell : cells) {
    if (numSet < semiStaticLimit) {
      entryMatrix(cell.y, cell.x) = 0.2 + perturb(rng);
      exitMatrix(cell.y, cell.x) = 0.2 + perturb(rng);
      initialBelief(cell.y, cell.x) = 0.5 + perturb(rng);
    } else if (numSet < dynamicLimit) {
      entryMatrix(cell.y, cell.x) = 0.5 + perturb(rng);
      exitMatrix(cell.y, cell.x) = 0.5 + perturb(rng);
      initialBelief(cell.y, cell.x) = 0.5 + perturb(rng);
    } else { // static free
      entryMatrix(cell.y, cell.x) = 0.0;
      exitMatrix(cell.y, cell.x) = 1.0;
      initialBelief(cell.y, cell.x) = 0.0;
    }
    ++numSet;

    if (cell == GridCell{0, 0}) { // Ensure robot init position is free
      initialBelief(cell.y, cell.x) = 0.0;
    }
  }

  return std::make_shared<IMac>(entryMatrix, exitMatrix, initialBelief);
}

/**
 * Generate a number of runs through an IMac model and write to file.
 *
 * @param imac The imac model
 * @param dir The directory to write to
 * @param timeBound The time bound for planning
 * @param numRuns The number of runs to sample
 */
void sampleRuns(std::shared_ptr<IMac> imac, const std::filesystem::path &dir,
                const int &timeBound, const int &numRuns) {

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  for (int run{1}; run <= numRuns; ++run) {
    std::cout << "Generating run " << run << "/" << numRuns << "\n";
    exec->restart(std::vector<IMacObservation>{});
    for (int t{1}; t <= timeBound; ++t) {
      exec->updateState(std::vector<IMacObservation>{});
    }
    exec->logMapDynamics(dir / ("run_" + std::to_string(run) + ".csv"));
  }
}

int main() {
  // Four by four w/ light dynamics
  std::cout << "Four by four w/ light dynamics\n";
  std::shared_ptr<IMac> fourLight{createIMac(4, 4, 0.1, 0.1)};
  sampleRuns(fourLight, "../../data/prelim_exps/four_light", 25, 10);

  // Four by four w/ heavy dynamics
  std::cout << "Four by four w/ heavy dynamics\n";
  std::shared_ptr<IMac> fourHeavy{createIMac(4, 4, 0.2, 0.2)};
  sampleRuns(fourHeavy, "../../data/prelim_exps/four_heavy", 25, 10);

  // Five by five w/ light dynamics
  std::cout << "Five by five w/ light dynamics\n";
  std::shared_ptr<IMac> fiveLight{createIMac(5, 5, 0.1, 0.1)};
  sampleRuns(fiveLight, "../../data/prelim_exps/five_light", 40, 10);

  // Five by five w/ heavy dynamics
  std::cout << "Five by five w/ heavy dynamics\n";
  std::shared_ptr<IMac> fiveHeavy{createIMac(5, 5, 0.2, 0.2)};
  sampleRuns(fiveHeavy, "../../data/prelim_exps/five_heavy", 40, 10);
}