/**
 * Example for IMac class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <iostream>

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

  Eigen::MatrixXd initBelief{2, 2};
  initBelief(0, 0) = 0.5;
  initBelief(0, 1) = 0.5;
  initBelief(1, 0) = 0.5;
  initBelief(1, 1) = 0.5;

  Eigen::MatrixXd currentBelief{2, 2};
  currentBelief(0, 0) = 0.1;
  currentBelief(0, 1) = 0.3;
  currentBelief(1, 0) = 0.5;
  currentBelief(1, 1) = 0.7;

  std::unique_ptr<IMac> imac{std::make_unique<IMac>(entry, exit, initBelief)};

  std::cout << "STATIC OCCUPANCY ESTIMATE\n";
  std::cout << imac->estimateStaticOccupancy() << '\n';
  std::cout << "NEXT STATE\n";
  std::cout << imac->forwardStep(currentBelief) << '\n';

  return 0;
}