/* Header file defining the IMac class.

IMac is a map of dynamics which uses an independent Markov chain at each grid
cell.

Author: Charlie Street
Owner: Charlie Street

*/
#ifndef IMAC_H
#define IMAC_H

#include <Eigen/Dense>
#include <memory>

/* Interactive Markov chain map of dynamics (IMac).
 * This class contains a 3D matrix which represents 2 state Markov chains at
 * every cell on a grid.
 *
 * Members:
 *  _transitionMatrix: A 3D transitionMatrix, where _transitionMatrix(x,y,0)
 * represents lambda_entry (free->occupied), and _transitionMatrix(x,y,1)
 * represents lambda_exit (occupied->free). _initialBelief: A 2D matrix
 * representing the initial belief over each cell being occupied.
 */
class IMac {
private:
  std::unique_ptr<Eigen::MatrixXd> _transitionMatrix{};
  std::unique_ptr<Eigen::MatrixXd> _initialBelief{};

public:
  /* Constructor initialises the member variables and asserts _transitionMatrix
   * is 3D.
   * Args:
   *  transitionMatrix: A ptr to a transition matrix
   */
  IMac(std::unique_ptr<Eigen::MatrixXd> transitionMatrix)
      : _transitionMatrix{std::move(transitionMatrix)}, _initialBelief{} {}

  std::unique_ptr<Eigen::MatrixXd> computeInitialBelief();

  std::unique_ptr<Eigen::MatrixXd>
  forwardStep(std::unique_ptr<Eigen::MatrixXd> currentState);
};

#endif