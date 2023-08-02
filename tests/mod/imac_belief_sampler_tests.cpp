/**
 * Unit tests for IMacBeliefSampler.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac_belief_sampler.h"
#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <vector>

TEST_CASE("Test deprecated functions.", "[IMacBeliefSampler::deprecated]") {
  IMacBeliefSampler sampler{};

  REQUIRE_THROWS(sampler.restart());
  REQUIRE_THROWS(sampler.logMapDynamics("file.csv"));
  REQUIRE_THROWS(sampler.updateState(std::vector<IMacObservation>{}));
  REQUIRE_THROWS(sampler.clearRobotPosition(GridCell{0, 0}));
}

TEST_CASE("Test sampleFromBelief.", "[IMacBeliefSampler::sampleFromBelief]") {
  IMacBeliefSampler sampler{};

  Eigen::MatrixXd distOne{2, 2};
  distOne(0, 0) = 0.5;
  distOne(0, 1) = 0.5;
  distOne(1, 0) = 0.5;
  distOne(1, 1) = 0.5;

  // Test seeding
  Eigen::MatrixXi matOne{sampler.sampleFromBelief(distOne, 0.5)};
  Eigen::MatrixXi matTwo{sampler.sampleFromBelief(distOne, 0.5)};

  REQUIRE(matOne(0, 0) == matTwo(0, 0));
  REQUIRE(matOne(0, 1) == matTwo(0, 1));
  REQUIRE(matOne(1, 0) == matTwo(1, 0));
  REQUIRE(matOne(1, 1) == matTwo(1, 1));

  // Test different when not same seed
  Eigen::MatrixXi matThree{sampler.sampleFromBelief(distOne)};

  int numSame{0};
  for (int i{0}; i < 2; ++i) {
    for (int j{0}; j < 2; ++j) {
      if (matOne(i, j) == matThree(i, j)) {
        ++numSame;
      }
    }
  }
  REQUIRE(numSame != 4);

  // Test observations
  std::vector<IMacObservation> imacObs{
      IMacObservation{GridCell{0, 0}, 1}, IMacObservation{GridCell{0, 1}, 1},
      IMacObservation{GridCell{1, 0}, 1}, IMacObservation{GridCell{1, 1}, 1}};
  Eigen::MatrixXi matFour{sampler.sampleFromBelief(distOne, 0.0, imacObs)};
  REQUIRE(matFour(0, 0) == 1);
  REQUIRE(matFour(0, 1) == 1);
  REQUIRE(matFour(1, 0) == 1);
  REQUIRE(matFour(1, 1) == 1);
}