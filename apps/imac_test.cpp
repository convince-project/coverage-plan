/**
 * Test of IMac class.
 *
 * @author Charlie Street
 */

#include "coverage-plan/mod/imac.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>

int main() {

  Eigen::MatrixXd entry{2, 2};

  entry(0, 0) = 0.2;
  entry(0, 1) = 0.3;
  entry(1, 0) = 0.4;
  entry(1, 1) = 0.5;

  Eigen::MatrixXd exit{2, 2};

  exit(0, 0) = 0.4;
  exit(0, 1) = 0.5;
  exit(1, 0) = 0.6;
  exit(1, 1) = 0.7;

  Eigen::MatrixXd currentState{2, 2};
  currentState(0, 0) = 0.1;
  currentState(0, 1) = 0.3;
  currentState(1, 0) = 0.5;
  currentState(1, 1) = 0.7;

  std::unique_ptr<IMac> imac{std::make_unique<IMac>(entry, exit)};

  std::cout << "INITIAL BELIEF\n";
  std::cout << imac->computeInitialBelief() << '\n';
  std::cout << "NEXT STATE\n";
  std::cout << imac->forwardStep(currentState) << '\n';

  return 0;
}