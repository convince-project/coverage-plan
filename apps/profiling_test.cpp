/**
 * Small script to help profile expensive parts of the coverage planner.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_belief_sampler.h"
#include "coverage_plan/util/seed.h"
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <stdlib.h>

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

/**
 * Time the performance of different RNGs.
 */
void profileRNG() {

  // Test 1: mt19937_64
  std::mt19937_64 gen64{SeedHelpers::genRandomDeviceSeed()};

  std::uniform_real_distribution<> dist64{0, 1};

  auto start{std::chrono::high_resolution_clock::now()};
  for (int i{0}; i < 1000000000; ++i) {
    double num{dist64(gen64)};
  }
  auto stop{std::chrono::high_resolution_clock::now()};
  auto duration{
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start)};
  std::cout << "MT19937_64 - Time elapsed: " << duration.count()
            << " microseconds\n";

  // Test 2: mt19937
  std::mt19937 gen32{SeedHelpers::genRandomDeviceSeed()};

  std::uniform_real_distribution<> dist32{0, 1};

  start = std::chrono::high_resolution_clock::now();
  for (int i{0}; i < 1000000000; ++i) {
    double num{dist32(gen32)};
  }
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "MT19937 - Time elapsed: " << duration.count()
            << " microseconds\n";

  // Test 3: rand
  srand(SeedHelpers::genRandomDeviceSeed());
  start = std::chrono::high_resolution_clock::now();
  for (int i{0}; i < 1000000000; ++i) {

    int num{rand()};
  }
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "rand - Time elapsed: " << duration.count() << " microseconds\n";

  // Test 4: rand_r
  unsigned int seed{SeedHelpers::genRandomDeviceSeed()};
  start = std::chrono::high_resolution_clock::now();
  for (int i{0}; i < 1000000000; ++i) {
    seed = rand_r(&seed);
  }
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "rand_r - Time elapsed: " << duration.count()
            << " microseconds\n";

  // Test 5: linear_congruential_engine
  std::linear_congruential_engine<std::uint_fast32_t, 48271, 0, 2147483647>
      genLcg{SeedHelpers::genRandomDeviceSeed()};

  std::uniform_real_distribution<> distLcg{0, 1};

  start = std::chrono::high_resolution_clock::now();
  for (int i{0}; i < 1000000000; ++i) {
    double num{distLcg(genLcg)};
  }
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "LCG - Time elapsed: " << duration.count() << " microseconds\n";

  // Test 6: Subtract with carry
  std::ranlux48 genSwc{SeedHelpers::genRandomDeviceSeed()};

  std::uniform_real_distribution<> distSwc{0, 1};

  start = std::chrono::high_resolution_clock::now();
  for (int i{0}; i < 1000000000; ++i) {
    double num{distSwc(genSwc)};
  }
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "SWC - Time elapsed: " << duration.count() << " microseconds\n";
}

int main() { profileMapUpdate(); }