/**
 * Tests for the IMacExecutor class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <memory>

TEST_CASE("Tests for restart function in IMacExecutor", "[restart]") {

  // Part 1: Test when every cell has the same probability (randomness test)
  Eigen::MatrixXd entryAndExit{Eigen::MatrixXd::Constant(3, 3, 0.2)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndExit, entryAndExit)};

  // Here, the IMacExecutor will take ownership of the imac pointer
  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  Eigen::MatrixXd initState = exec->restart();

  // Check matrix is of expected size and all values are 0 or 1
  REQUIRE(initState.rows() == 3);
  REQUIRE(initState.cols() == 3);
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      REQUIRE((initState(i, j) == 0.0 || initState(i, j) == 1.0));
    }
  }

  // Part 2: Run again with same matrix and check values are different
  Eigen::MatrixXd initStateTwo{exec->restart()};

  // Sanity check second matrix
  REQUIRE(initStateTwo.rows() == 3);
  REQUIRE(initStateTwo.cols() == 3);
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      REQUIRE((initStateTwo(i, j) == 0.0 || initStateTwo(i, j) == 1.0));
    }
  }

  // Check the values are different
  int numSame{0};
  for (int i{0}; i < 2; ++i) {
    for (int j{0}; j < 2; ++j) {
      if (initState(i, j) == initStateTwo(i, j)) {
        ++numSame;
      }
    }
  }

  // This test will occasionally fail. This is to be expected.
  // Re-run and it should be fine
  REQUIRE(numSame != 9);

  // Part 3: Try with deterministic initial belief
  Eigen::MatrixXd detEntry{2, 2};
  detEntry(0, 0) = 1;
  detEntry(0, 1) = 0;
  detEntry(1, 0) = 0;
  detEntry(1, 1) = 1;

  Eigen::MatrixXd detExit{2, 2};
  detExit(0, 0) = 0;
  detExit(0, 1) = 1;
  detExit(1, 0) = 1;
  detExit(1, 1) = 0;

  // Memory allocation shouldn't be an issue here
  imac = std::make_shared<IMac>(detEntry, detExit);
  exec = std::make_unique<IMacExecutor>(imac);

  initState = exec->restart();
  // Check the values are 1,0,0,1 - checking the computations are the right way
  // round
  REQUIRE(initState(0, 0) == 1.0);
  REQUIRE(initState(0, 1) == 0.0);
  REQUIRE(initState(1, 0) == 0.0);
  REQUIRE(initState(1, 1) == 1.0);
}