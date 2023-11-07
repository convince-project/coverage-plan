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
 * Create a random IMac instance with jmore complex dynamics
 *
 * @param xDim X dimension of the map
 * @param yDim Y dimension of the map
 * @param propSlow Proportion of the map to have slow dynamics (0.05)
 * @param propMed Proportion of the map to have medium dynamics (0.2)
 * @param propFast Proportion of the map to have fast dynamics (0.1)
 *
 * 1-(propSlow+propMed+propFast) of the map is free
 *
 * @return imac A shared pointer to an IMac instance
 */
std::shared_ptr<IMac> createMoreComplexIMac(const int &xDim, const int &yDim,
                                            const double &propSlow,
                                            const double &propMed,
                                            const double &propFast) {
  Eigen::MatrixXd entryMatrix{yDim, xDim};
  Eigen::MatrixXd exitMatrix{yDim, xDim};
  Eigen::MatrixXd initialBelief{yDim, xDim};

  int numSet{0};
  const int slowLimit{(int)ceil(propSlow * (xDim * yDim))};
  const int medLimit{(int)ceil(slowLimit + (propMed * (xDim * yDim)))};
  const int fastLimit{(int)ceil(medLimit + (propFast * (xDim * yDim)))};

  std::cout << "(Free, Fast, Med, Slow): (" << ((xDim * yDim) - fastLimit)
            << ", " << fastLimit - medLimit << ", " << medLimit - slowLimit
            << ", " << slowLimit << ")\n";

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
    if (numSet < slowLimit) {
      entryMatrix(cell.y, cell.x) = 0.05 + perturb(rng);
      exitMatrix(cell.y, cell.x) = 0.05 + perturb(rng);
      initialBelief(cell.y, cell.x) = 0.5 + perturb(rng);
    } else if (numSet < medLimit) {
      entryMatrix(cell.y, cell.x) = 0.2 + perturb(rng);
      exitMatrix(cell.y, cell.x) = 0.2 + perturb(rng);
      initialBelief(cell.y, cell.x) = 0.5 + perturb(rng);
    } else if (numSet < fastLimit) {
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
            << ((xDim * yDim) - dynamicLimit) << ", " << semiStaticLimit << ", "
            << (dynamicLimit - semiStaticLimit) << ")\n";

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

  // Nine by nine with 20,30,10,40 (slow, med, fast, static)
  std::cout << "9x9 with 20 slow, 30 med, 10 fast, 40 slow\n";
  std::shared_ptr<IMac> nineVeryHeavy{
      createMoreComplexIMac(9, 9, 0.2, 0.3, 0.1)};
  std::string nineVeryHeavyDir{"../../data/prelim_exps/nine_very_heavy"};
  nineVeryHeavy->writeIMac(nineVeryHeavyDir);
  sampleRuns(nineVeryHeavy, nineVeryHeavyDir, 106, 40);
  exit(1);

  // Eight by eight with 20,30,10,40 (slow, med, fast, static)
  std::cout << "8x8 with 20 slow, 30 med, 10 fast, 40 slow\n";
  std::shared_ptr<IMac> eightVeryHeavy{
      createMoreComplexIMac(8, 8, 0.2, 0.3, 0.1)};
  std::string eightVeryHeavyDir{"../../data/prelim_exps/eight_very_heavy"};
  eightVeryHeavy->writeIMac(eightVeryHeavyDir);
  sampleRuns(eightVeryHeavy, eightVeryHeavyDir, 84, 40);
  exit(1);

  // Ten by ten with 20,30,10,40 (slow, med, fast, static)
  std::cout << "10x10 with 20 slow, 30 med, 10 fast, 40 slow\n";
  std::shared_ptr<IMac> tenVeryHeavy{
      createMoreComplexIMac(10, 10, 0.2, 0.3, 0.1)};
  std::string tenVeryHeavyDir{"../../data/prelim_exps/ten_very_heavy"};
  tenVeryHeavy->writeIMac(tenVeryHeavyDir);
  sampleRuns(tenVeryHeavy, tenVeryHeavyDir, 130, 40);
  exit(1);

  // Creating two environments with more intense dynamics
  // Five by five with 20,30,10,40 (slow, med, fast, static)
  std::cout << "5x5 with 20 slow, 30 med, 10 fast, 40 slow\n";
  std::shared_ptr<IMac> fiveVeryHeavy{
      createMoreComplexIMac(5, 5, 0.2, 0.3, 0.1)};
  std::string fiveVeryHeavyDir{"../../data/prelim_exps/five_very_heavy"};
  fiveVeryHeavy->writeIMac(fiveVeryHeavyDir);
  sampleRuns(fiveVeryHeavy, fiveVeryHeavyDir, 33, 40);

  // Seven by seven with 20,30,10,40 (slow, med, fast, static)
  std::cout << "7x7 with 20 slow, 30 med, 10 fast, 40 slow\n";
  std::shared_ptr<IMac> sevenVeryHeavy{
      createMoreComplexIMac(7, 7, 0.2, 0.3, 0.1)};
  std::string sevenVeryHeavyDir{"../../data/prelim_exps/seven_very_heavy"};
  sevenVeryHeavy->writeIMac(sevenVeryHeavyDir);
  sampleRuns(sevenVeryHeavy, sevenVeryHeavyDir, 64, 40);
  exit(1);

  // Five by five w/ lots of semi-static obstacles
  std::cout << "Five by five w/ lots of semi-static s\n";
  std::shared_ptr<IMac> fiveSemi{createIMac(5, 5, 0.4, 0.0)};
  std::string fiveSemiDir{"../../data/prelim_exps/five_semi_static"};
  fiveSemi->writeIMac(fiveSemiDir);
  sampleRuns(fiveSemi, fiveSemiDir, 40, 10);
  exit(1);

  // Four by four w/ light dynamics
  std::cout << "Four by four w/ light dynamics\n";
  std::shared_ptr<IMac> fourLight{createIMac(4, 4, 0.1, 0.1)};
  std::string fourLightDir{"../../data/prelim_exps/four_light"};
  fourLight->writeIMac(fourLightDir);
  sampleRuns(fourLight, fourLightDir, 25, 10);

  // Four by four w/ heavy dynamics
  std::cout << "Four by four w/ heavy dynamics\n";
  std::shared_ptr<IMac> fourHeavy{createIMac(4, 4, 0.2, 0.2)};
  std::string fourHeavyDir{"../../data/prelim_exps/four_heavy"};
  fourHeavy->writeIMac(fourHeavyDir);
  sampleRuns(fourHeavy, fourHeavyDir, 25, 10);

  // Five by five w/ light dynamics
  std::cout << "Five by five w/ light dynamics\n";
  std::shared_ptr<IMac> fiveLight{createIMac(5, 5, 0.1, 0.1)};
  std::string fiveLightDir{"../../data/prelim_exps/five_light"};
  fiveLight->writeIMac(fiveLightDir);
  sampleRuns(fiveLight, fiveLightDir, 40, 10);

  // Five by five w/ heavy dynamics
  std::cout << "Five by five w/ heavy dynamics\n";
  std::shared_ptr<IMac> fiveHeavy{createIMac(5, 5, 0.2, 0.2)};
  std::string fiveHeavyDir{"../../data/prelim_exps/five_heavy"};
  fiveHeavy->writeIMac(fiveHeavyDir);
  sampleRuns(fiveHeavy, fiveHeavyDir, 40, 10);
}