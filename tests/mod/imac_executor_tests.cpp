/**
 * Tests for the IMacExecutor class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

TEST_CASE("Tests for restart function in IMacExecutor", "[restart]") {

  // Part 1: Test when every cell has the same probability (randomness test)
  Eigen::MatrixXd entryAndExit{Eigen::MatrixXd::Constant(3, 3, 0.2)};
  Eigen::MatrixXd initBelief{Eigen::MatrixXd::Constant(3, 3, 0.5)};

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryAndExit, entryAndExit, initBelief)};

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  Eigen::MatrixXi initState{exec->restart()};

  // Check matrix is of expected size and all values are 0 or 1
  REQUIRE(initState.rows() == 3);
  REQUIRE(initState.cols() == 3);
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      REQUIRE((initState(i, j) == 0 || initState(i, j) == 1));
    }
  }

  // Part 2: Run again with same matrix and check values are different
  Eigen::MatrixXi initStateTwo{exec->restart()};

  // Sanity check second matrix
  REQUIRE(initStateTwo.rows() == 3);
  REQUIRE(initStateTwo.cols() == 3);
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      REQUIRE((initStateTwo(i, j) == 0 || initStateTwo(i, j) == 1));
    }
  }

  // Check the values are different
  int numSame{0};
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      if (initState(i, j) == initStateTwo(i, j)) {
        ++numSame;
      }
    }
  }

  // This test will occasionally fail. This is to be expected.
  // Re-run and it should be fine
  REQUIRE(numSame != 9);

  // Part 3: Check with initial observations
  std::vector<IMacObservation> initialObs{};
  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      initialObs.push_back(IMacObservation{GridCell{i, j}, 1});
    }
  }

  initState = exec->restart(initialObs);

  for (int i{0}; i < 3; ++i) {
    for (int j{0}; j < 3; ++j) {
      REQUIRE(initState(i, j) == 1);
    }
  }

  // Part 4: Try with deterministic initial belief
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

  Eigen::MatrixXd detInit{2, 2};
  detInit(0, 0) = 1;
  detInit(0, 1) = 0;
  detInit(1, 0) = 0;
  detInit(1, 1) = 1;

  // Memory allocation shouldn't be an issue here
  imac = std::make_shared<IMac>(detEntry, detExit, detInit);
  exec = std::make_unique<IMacExecutor>(imac);

  initState = exec->restart();
  // Check the values are 1,0,0,1 - checking the computations are the right way
  // round
  REQUIRE(initState(0, 0) == 1);
  REQUIRE(initState(0, 1) == 0);
  REQUIRE(initState(1, 0) == 0);
  REQUIRE(initState(1, 1) == 1);
}

TEST_CASE("Tests for the updateState function in IMacExecutor",
          "[updateState]") {

  // Create an iMac instant with highly random dynamics
  Eigen::MatrixXd entryExitAndInit{Eigen::MatrixXd::Constant(3, 3, 0.5)};

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(
      entryExitAndInit, entryExitAndInit, entryExitAndInit)};

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  // Set _currentState to something internally
  exec->restart();

  // Step 1: Check that the observation masking works
  std::vector<IMacObservation> obs{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      obs.push_back(IMacObservation{GridCell{i, j}, 1});
    }
  }

  Eigen::MatrixXi nextState{exec->updateState(obs)};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      REQUIRE(nextState(i, j) == 1);
    }
  }

  // Step 2: Check that the next state is different
  obs.clear();
  Eigen::MatrixXi nextStateTwo(exec->updateState(obs));
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
  Eigen::MatrixXi nextStateThree(exec->updateState(obs));
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
  Eigen::MatrixXd detEntryAndInit{Eigen::MatrixXd::Constant(3, 3, 1)};
  Eigen::MatrixXd detExit{Eigen::MatrixXd::Constant(3, 3, 0)};

  // Memory allocation shouldn't be an issue here
  imac = std::make_shared<IMac>(detEntryAndInit, detExit, detEntryAndInit);
  exec = std::make_unique<IMacExecutor>(imac);

  Eigen::MatrixXi initState{exec->restart()};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      REQUIRE(initState(i, j) == 1);
    }
  }

  obs.push_back(IMacObservation{GridCell{1, 1}, 0});
  nextState = exec->updateState(obs);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == 1 && j == 1) {
        REQUIRE(nextState(i, j) == 0);
      } else {
        REQUIRE(nextState(i, j) == 1);
      }
    }
  }
}

TEST_CASE("Tests for the logMapDynamics function in IMacExecutor",
          "[logMapDynamics]") {
  Eigen::MatrixXd entryExitAndInit{Eigen::MatrixXd::Constant(2, 2, 0.5)};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(
      entryExitAndInit, entryExitAndInit, entryExitAndInit)};

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  // Fixing map state for example
  std::vector<IMacObservation> obsZero{
      IMacObservation{GridCell{0, 0}, 1}, IMacObservation{GridCell{1, 0}, 0},
      IMacObservation{GridCell{0, 1}, 0}, IMacObservation{GridCell{1, 1}, 1}};

  std::vector<IMacObservation> obsOne{
      IMacObservation{GridCell{0, 0}, 0}, IMacObservation{GridCell{1, 0}, 1},
      IMacObservation{GridCell{0, 1}, 1}, IMacObservation{GridCell{1, 1}, 0}};

  std::vector<IMacObservation> obsTwo{
      IMacObservation{GridCell{0, 0}, 1}, IMacObservation{GridCell{1, 0}, 1},
      IMacObservation{GridCell{0, 1}, 0}, IMacObservation{GridCell{1, 1}, 0}};

  exec->restart(obsZero);
  exec->updateState(obsOne);
  exec->updateState(obsTwo);

  exec->logMapDynamics("/tmp/mapLogTest.csv");

  // Read file back in as vector of elements
  std::vector<int> mapElems;
  std::ifstream f("/tmp/mapLogTest.csv");

  std::string rowString;
  std::string entryString;

  int numRows{0};

  while (getline(f, rowString)) {
    std::stringstream rowStream(rowString);
    while (getline(rowStream, entryString, ',')) {
      mapElems.push_back(std::stoi(entryString));
    }
    ++numRows;
  }

  std::vector<int> expected{0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1,
                            1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0,
                            2, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0};

  // Length == 39
  REQUIRE(mapElems.size() == expected.size());

  for (int i{0}; i < mapElems.size(); ++i) {
    REQUIRE(mapElems.at(i) == expected.at(i));
  }
}

TEST_CASE("Tests for clearRobotPosition in IMacExecutor",
          "[clearRobotPosition]") {
  // Create an iMac instant with dynamics which never change
  Eigen::MatrixXd entryExit{Eigen::MatrixXd::Constant(1, 1, 0.0)};

  // initially occupied
  Eigen::MatrixXd init{1, 1};
  init(0, 0) = 1;

  std::shared_ptr<IMac> imac{
      std::make_shared<IMac>(entryExit, entryExit, init)};

  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  Eigen::MatrixXi initState{exec->restart()};

  REQUIRE(initState(0, 0) == 1);

  exec->clearRobotPosition(GridCell{0, 0});

  Eigen::MatrixXi nextState{exec->updateState(std::vector<IMacObservation>{})};
  // Can only be correct if the clearing has worked
  REQUIRE(nextState(0, 0) == 0);

  exec->logMapDynamics("/tmp/clearTest.csv");

  // Read file back in as vector of elements
  std::vector<int> mapElems;
  std::ifstream f("/tmp/clearTest.csv");

  std::string rowString;
  std::string entryString;

  int numRows{0};

  while (getline(f, rowString)) {
    std::stringstream rowStream(rowString);
    while (getline(rowStream, entryString, ',')) {
      mapElems.push_back(std::stoi(entryString));
    }
    ++numRows;
  }

  std::vector<int> expected{0, 0, 0, 0, 1, 0, 0, 0};

  // Length == 8
  REQUIRE(mapElems.size() == expected.size());

  for (int i{0}; i < mapElems.size(); ++i) {
    REQUIRE(mapElems.at(i) == expected.at(i));
  }
}