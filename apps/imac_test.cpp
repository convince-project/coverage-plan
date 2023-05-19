/**
 * Test of IMac class.
 *
 * @author Charlie Street
 */

#include "coverage-plan/mod/imac.h"
#include <Eigen/Dense>
#include <memory>

int main() {

  std::unique_ptr<Eigen::MatrixXd> entry{
      std::make_shared<Eigen::MatrixXd>(2, 2)};

  (*entry)(0, 0) = 0.2;
  (*entry)(0, 1) = 0.3;
  (*entry)(1, 0) = 0.4;
  (*entry)(1, 1) = 0.5;

  std::unique_ptr<Eigen::MatrixXd> exit{
      std::make_shared<Eigen::MatrixXd>(2, 2)};

  (*exit)(0, 0) = 0.4;
  (*exit)(0, 1) = 0.5;
  (*exit)(1, 0) = 0.6;
  (*exit)(1, 1) = 0.7;

  std::unique_ptr<Eigen::MatrixXd> currentState{
      std::make_shared<Eigen::MatrixXd>(2, 2)};
  (*currentState)(0, 0) = 0.1;
  (*currentState)(0, 1) = 0.3;
  (*currentState)(1, 0) = 0.5;
  (*currentState)(1, 1) = 0.7;

  IMac imac{entry, exit};

  std::cout << "INITIAL BELIEF\n";
  std::cout << *(imac->computeInitialBelief()) << '\n';
  std::cout << "NEXT STATE\n";
  std::cout << *(imac->forwardStep()) << '\n';

  return 0;
}