/**
 * Tests for the IMac class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>

TEST_CASE("Tests for basic IMac functionality", "[imac]") {

  // Entry matrix
  Eigen::MatrixXd entry{2, 2};
  entry(0, 0) = 0.2;
  entry(0, 1) = 0.3;
  entry(1, 0) = 0.4;
  entry(1, 1) = 0.5;

  // Exit matrix
  Eigen::MatrixXd exit{2, 2};
  exit(0, 0) = 0.4;
  exit(0, 1) = 0.5;
  exit(1, 0) = 0.6;
  exit(1, 1) = 0.7;

  // Initial belief
  Eigen::MatrixXd initBelief{2, 2};
  initBelief(0, 0) = 0.5;
  initBelief(0, 1) = 0.6;
  initBelief(1, 0) = 0.7;
  initBelief(1, 1) = 0.8;

  // An example mod belief
  Eigen::MatrixXd currentBelief{2, 2};
  currentBelief(0, 0) = 0.1;
  currentBelief(0, 1) = 0.3;
  currentBelief(1, 0) = 0.5;
  currentBelief(1, 1) = 0.7;

  // Create the IMac object
  std::unique_ptr<IMac> imac{std::make_unique<IMac>(entry, exit, initBelief)};

  auto staticOcc{imac->estimateStaticOccupancy()};
  auto nextState{imac->forwardStep(currentBelief)};

  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.4, 0.001));

  REQUIRE_THAT(nextState(0, 0), Catch::Matchers::WithinRel(0.24, 0.001));
  REQUIRE_THAT(nextState(0, 1), Catch::Matchers::WithinRel(0.36, 0.001));
  REQUIRE_THAT(nextState(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(nextState(1, 1), Catch::Matchers::WithinRel(0.36, 0.001));

  // Test getters
  Eigen::MatrixXd entryGot{imac->getEntryMatrix()};
  Eigen::MatrixXd exitGot{imac->getExitMatrix()};
  Eigen::MatrixXd initGot{imac->getInitialBelief()};

  REQUIRE_THAT(entryGot(0, 0), Catch::Matchers::WithinRel(0.2, 0.001));
  REQUIRE_THAT(entryGot(0, 1), Catch::Matchers::WithinRel(0.3, 0.001));
  REQUIRE_THAT(entryGot(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(entryGot(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));

  REQUIRE_THAT(exitGot(0, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(exitGot(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(exitGot(1, 0), Catch::Matchers::WithinRel(0.6, 0.001));
  REQUIRE_THAT(exitGot(1, 1), Catch::Matchers::WithinRel(0.7, 0.001));

  REQUIRE_THAT(initGot(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initGot(0, 1), Catch::Matchers::WithinRel(0.6, 0.001));
  REQUIRE_THAT(initGot(1, 0), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initGot(1, 1), Catch::Matchers::WithinRel(0.8, 0.001));
}