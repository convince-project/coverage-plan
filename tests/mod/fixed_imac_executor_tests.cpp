/**
 * Unit tests for FixedIMacExecutor in fixed_imac_executor.h/.cpp.
 * @see fixed_imac_executor.h/.cpp
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/fixed_imac_executor.h"
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <filesystem>
#include <memory>

TEST_CASE("Test for FixedIMacExecutor", "[FixedIMacExecutor]") {
  Eigen::MatrixXd imacMat{Eigen::MatrixXd::Constant(3, 2, 0.5)};
  std::shared_ptr<IMac> imac{std::make_shared<IMac>(imacMat, imacMat, imacMat)};

  std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};

  // Episodes go from time step 0 - 3 inclusive
  std::filesystem::path pathOne{"/tmp/episodeOne.csv"};
  std::vector<Eigen::MatrixXi> episodeOne{};
  episodeOne.push_back(exec->restart());
  episodeOne.push_back(exec->updateState(std::vector<IMacObservation>{}));
  episodeOne.push_back(exec->updateState(std::vector<IMacObservation>{}));
  episodeOne.push_back(exec->updateState(std::vector<IMacObservation>{}));
  exec->logMapDynamics(pathOne);

  std::filesystem::path pathTwo{"/tmp/episodeTwo.csv"};
  std::vector<Eigen::MatrixXi> episodeTwo{};
  episodeTwo.push_back(exec->restart());
  episodeTwo.push_back(exec->updateState(std::vector<IMacObservation>{}));
  episodeTwo.push_back(exec->updateState(std::vector<IMacObservation>{}));
  episodeTwo.push_back(exec->updateState(std::vector<IMacObservation>{}));
  exec->logMapDynamics(pathTwo);

  // Now create the fixed IMac Executor
  std::vector<std::filesystem::path> files{pathOne, pathTwo};
  std::shared_ptr<FixedIMacExecutor> fixedExec{
      std::make_shared<FixedIMacExecutor>(files, 2, 3)};

  // Should be equal to episodeOne
  std::vector<Eigen::MatrixXi> fixedOne{};
  fixedOne.push_back(fixedExec->restart());
  fixedOne.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedOne.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedOne.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));

  // Should be equal to episodeTwo
  std::vector<Eigen::MatrixXi> fixedTwo{};
  fixedTwo.push_back(fixedExec->restart());
  fixedTwo.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedTwo.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedTwo.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));

  // Should be equal to episodeOne
  std::vector<Eigen::MatrixXi> fixedThree{};
  fixedThree.push_back(fixedExec->restart());
  fixedThree.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedThree.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedThree.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));

  // Should be equal to episodeTwo
  std::vector<Eigen::MatrixXi> fixedFour{};
  fixedFour.push_back(fixedExec->restart());
  fixedFour.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedFour.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));
  fixedFour.push_back(fixedExec->updateState(std::vector<IMacObservation>{}));

  // Going beyond the length of the episode, should throw exception
  REQUIRE_THROWS(fixedExec->updateState(std::vector<IMacObservation>{}));

  // Now check the fixed executions to the original episodes
  for (int ts{0}; ts <= 3; ++ts) {
    for (int i{0}; i < 3; ++i) {
      for (int j{0}; j < 2; ++j) {
        REQUIRE(fixedOne.at(ts)(i, j) == episodeOne.at(ts)(i, j));
        REQUIRE(fixedTwo.at(ts)(i, j) == episodeTwo.at(ts)(i, j));
        REQUIRE(fixedThree.at(ts)(i, j) == episodeOne.at(ts)(i, j));
        REQUIRE(fixedFour.at(ts)(i, j) == episodeTwo.at(ts)(i, j));
      }
    }
  }
}