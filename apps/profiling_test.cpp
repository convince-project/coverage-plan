/**
 * Small script to help profile expensive parts of the coverage planner.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_belief_sampler.h"
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>

/**
 * Time the map update code, as this is likely to be expensive.
 */
void profileMapUpdate() {
  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>("../../data/prelim_exps/five_heavy")};

  std::unique_ptr<IMacBeliefSampler> sampler{
      std::make_unique<IMacBeliefSampler>()};

  // Generate initial matrix from IMacExecutor
  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};
  Eigen::MatrixXi map{exec->restart()};

  // Time the test
  auto start{std::chrono::high_resolution_clock::now()};
  for (int i{0}; i < 1000000; ++i) {
    map = sampler->sampleFromBelief(imac->forwardStep(map.cast<double>()));
  }
  auto stop{std::chrono::high_resolution_clock::now()};
  auto duration{
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start)};

  std::cout << "Time elapsed: " << duration.count() << " microseconds\n";
}

int main() { profileMapUpdate(); }