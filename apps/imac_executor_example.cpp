/**
 * An example run through an IMac model using the IMacExecutor class.
 *
 * @author Charlie Street
 */
#include "coverage_plan/mod/imac.h"
#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

int main() {
  // Step 1: Create the IMac model
  Eigen::MatrixXd entry{Eigen::MatrixXd::Constant(3, 3, 0.5)};
  Eigen::MatrixXd exit{Eigen::MatrixXd::Constant(3, 3, 0.5)};

  // (0,0) and (0,1) are always occupied,
  entry(0, 0) = 1.0;
  entry(0, 1) = 1.0;
  entry(0, 2) = 0.0;
  entry(1, 0) = 0.0;

  exit(0, 0) = 0.0;
  exit(0, 1) = 0.0;
  exit(0, 2) = 1.0;
  exit(1, 0) = 1.0;

  std::shared_ptr<IMac> imac{std::make_shared<IMac>(entry, exit)};

  // Step 2: Make the iMac executor model
  std::unique_ptr<IMacExecutor> exec{std::make_unique<IMacExecutor>(imac)};

  // Step 3: Sample the initial state of the MoD in this sample run
  Eigen::MatrixXi currentState{exec->restart()};
  std::cout << "Time t=0\n";
  std::cout << currentState << '\n';

  // Step 4: Sample successor states with observations

  // t=1
  std::vector<IMacObservation> obs{};
  obs.push_back(IMacObservation{2, 2, 0});
  obs.push_back(IMacObservation{1, 2, 1});
  currentState = exec->updateState(obs);
  std::cout << "Time t=1\n";
  std::cout << currentState << '\n';

  // t=2
  obs.clear();
  obs.push_back(IMacObservation{1, 2, 0});
  obs.push_back(IMacObservation{1, 1, 1});
  currentState = exec->updateState(obs);
  std::cout << "Time t=2\n";
  std::cout << currentState << '\n';

  // t=3
  obs.clear();
  obs.push_back(IMacObservation{0, 2, 1});
  obs.push_back(IMacObservation{1, 2, 1});
  currentState = exec->updateState(obs);
  std::cout << "Time t=3\n";
  std::cout << currentState << '\n';

  // t=4
  obs.clear();
  obs.push_back(IMacObservation{1, 1, 0});
  obs.push_back(IMacObservation{2, 1, 0});
  currentState = exec->updateState(obs);
  std::cout << "Time t=4\n";
  std::cout << currentState << '\n';

  // t=5
  obs.clear();
  obs.push_back(IMacObservation{1, 1, 0});
  obs.push_back(IMacObservation{1, 2, 1});
  currentState = exec->updateState(obs);
  std::cout << "Time t=5\n";
  std::cout << currentState << '\n';

  // And we're done!
}