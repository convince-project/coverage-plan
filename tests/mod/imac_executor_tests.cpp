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
#include <tuple>
#include <vector>

TEST_CASE("Tests for restart function in IMacExecutor", "[restart]") {

  // Part 1: Test when every cell has the same probability (randomness test)
  Eigen::MatrixXd entryAndExit{Eigen::MatrixXd::Constant(3, 3, 0.2)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndExit, entryAndExit)};

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  Eigen::MatrixXd initState{exec->restart()};

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

TEST_CASE("Tests for the updateState function in IMacExecutor",
          "[updateState]") {

  // Create an iMac instant with highly random dynamics
  Eigen::MatrixXd entryAndExit{Eigen::MatrixXd::Constant(3, 3, 0.5)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndExit, entryAndExit)};

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  // Set _currentState to something internally
  exec->restart();

  // Step 1: Check that the observation masking works
  std::vector<std::tuple<int, int, int>> obs{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      obs.push_back(std::make_tuple(i, j, 1));
    }
  }

  Eigen::MatrixXd nextState{exec->updateState(obs)};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      REQUIRE(nextState(i, j) == 1.0);
    }
  }

  // Step 2: Check that the next state is different
  obs.clear();
  Eigen::MatrixXd nextStateTwo(exec->updateState(obs));
  int numSame{0};
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      if (nextState(i, j) == nextStateTwo(i, j)) {
        ++numSame;
      }
    }
  }
  // This test will occasionally fail. This is to be expected.
  // Re-run and it should be fine
  REQUIRE(numSame != 9);

  // Step 3: Check multiple runs without masking
  Eigen::MatrixXd nextStateThree(exec->updateState(obs));
  numSame = 0;
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      if (nextStateTwo(i, j) == nextStateThree(i, j)) {
        ++numSame;
      }
    }
  }
  // This test will occasionally fail. This is to be expected.
  // Re-run and it should be fine
  REQUIRE(numSame != 9);

  // Step 4: Test with deterministic iMac model (always occupied)
  Eigen::MatrixXd detEntry{Eigen::MatrixXd::Constant(3, 3, 1)};
  Eigen::MatrixXd detExit{Eigen::MatrixXd::Constant(3, 3, 0)};

  // Memory allocation shouldn't be an issue here
  imac = std::make_shared<IMac>(detEntry, detExit);
  exec = std::make_unique<IMacExecutor>(imac);

  Eigen::MatrixXd initState{exec->restart()};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      REQUIRE(initState(i, j) == 1.0);
    }
  }

  obs.push_back(std::make_tuple(1, 1, 0));
  nextState = exec->updateState(obs);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == 1 && j == 1) {
        REQUIRE(nextState(i, j) == 0.0);
      } else {
        REQUIRE(nextState(i, j) == 1.0);
      }
    }
  }
}