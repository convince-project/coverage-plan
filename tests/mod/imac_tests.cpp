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

  // An example mod belief
  Eigen::MatrixXd currentBelief{2, 2};
  currentBelief(0, 0) = 0.1;
  currentBelief(0, 1) = 0.3;
  currentBelief(1, 0) = 0.5;
  currentBelief(1, 1) = 0.7;

  // Create the IMac object
  std::unique_ptr<IMac> imac{std::make_unique<IMac>(entry, exit)};

  auto initialBelief{imac->computeInitialBelief()};
  auto nextState{imac->forwardStep(currentBelief)};

  REQUIRE_THAT(initialBelief(0, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(initialBelief(0, 1), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(initialBelief(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(initialBelief(1, 1), Catch::Matchers::WithinRel(0.4, 0.001));

  REQUIRE_THAT(nextState(0, 0), Catch::Matchers::WithinRel(0.24, 0.001));
  REQUIRE_THAT(nextState(0, 1), Catch::Matchers::WithinRel(0.36, 0.001));
  REQUIRE_THAT(nextState(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(nextState(1, 1), Catch::Matchers::WithinRel(0.36, 0.001));
}